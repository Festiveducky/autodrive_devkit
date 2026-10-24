"""Behavioral tests for watchdog deadlines, ownership, and recovery."""

import math
import unittest

from autodrive_roboracer.command_safety import CommandPublisher, CommandSafety


def writers(name='/controller', seed=b'a'):
    """Return separate DDS writers owned by one controller node."""
    return [CommandPublisher(name, seed + b't')], [CommandPublisher(name, seed + b's')]


class CommandSafetyTests(unittest.TestCase):
    """A failed channel invalidates the entire pair, not just its own axis."""

    def setUp(self):
        self.gate = CommandSafety(timeout=0.25)
        self.throttle, self.steering = writers()
        self.gate.set_publishers(self.throttle, self.steering)

    def send(self, throttle=0.12, steering=0.3, now=10.0, source_age=0.0):
        self.gate.receive('throttle', throttle, b'at', now, source_age)
        self.gate.receive('steering', steering, b'as', now, source_age)

    def test_initial_state_and_missing_pair_are_zero(self):
        self.assertFalse(CommandSafety().snapshot(10)['enabled'])
        self.gate.receive('throttle', 0.12, b'at', 10)
        state = self.gate.snapshot(10)
        self.assertEqual((state['throttle'], state['steering']), (0, 0))
        self.assertFalse(state['enabled'])

    def test_fresh_pair_and_rounding(self):
        self.send(throttle=0.12345, steering=-0.34567)
        state = self.gate.snapshot(10.1)
        self.assertTrue(state['enabled'])
        self.assertEqual((state['throttle'], state['steering']), (0.123, -0.346))
        self.assertEqual(state['owner'], '/controller')

    def test_timeout_zeroes_both_channels(self):
        self.send()
        self.assertTrue(self.gate.snapshot(10.25)['enabled'])
        state = self.gate.snapshot(10.251)
        self.assertFalse(state['enabled'])
        self.assertEqual(state['reason'], 'command timeout')
        self.assertEqual((state['throttle'], state['steering']), (0, 0))

    def test_one_channel_cannot_keep_other_alive(self):
        self.send()
        self.gate.receive('throttle', 0.12, b'at', 10.2)
        self.assertFalse(self.gate.snapshot(10.251)['enabled'])
        self.gate.receive('throttle', 0.12, b'at', 10.26)
        self.assertFalse(self.gate.snapshot(10.26)['enabled'])
        self.gate.receive('steering', 0.1, b'as', 10.27)
        self.assertTrue(self.gate.snapshot(10.27)['enabled'])

    def test_transport_delay_consumes_deadline(self):
        self.send(source_age=0.20)
        self.assertTrue(self.gate.snapshot(10.04)['enabled'])
        self.assertFalse(self.gate.snapshot(10.06)['enabled'])

    def test_expired_or_invalid_source_time_is_rejected(self):
        for age in (0.251, math.nan, math.inf, -0.1):
            self.send()
            self.assertFalse(self.gate.receive('throttle', 0.1, b'at', 10.1, age))
            self.assertFalse(self.gate.snapshot(10.1)['enabled'])

    def test_invalid_values_clear_previous_pair(self):
        for value in (math.nan, math.inf, -math.inf, 1.1, -1.1):
            self.send()
            self.assertFalse(self.gate.receive('steering', value, b'as', 10.1))
            self.assertFalse(self.gate.snapshot(10.1)['enabled'])
        self.assertEqual(self.gate.rejected_messages, 5)

    def test_wrong_writer_cannot_impersonate_owner(self):
        self.send()
        self.assertFalse(self.gate.receive('throttle', 1.0, b'foreign', 10.1))
        self.assertFalse(self.gate.snapshot(10.1)['enabled'])

    def test_competing_publishers_stop_without_sending(self):
        self.send()
        other_throttle, _ = writers('/other', b'b')
        self.gate.set_publishers(self.throttle + other_throttle, self.steering)
        state = self.gate.snapshot(10.1)
        self.assertFalse(state['enabled'])
        self.assertEqual(state['publisher_counts']['throttle'], 2)
        self.send(now=10.1)
        self.assertFalse(self.gate.snapshot(10.1)['enabled'])

    def test_different_nodes_cannot_supply_half_each(self):
        _, other_steering = writers('/other', b'b')
        self.gate.set_publishers(self.throttle, other_steering)
        self.assertIn('different owners', self.gate.snapshot(10)['reason'])

    def test_conflict_recovery_requires_new_pair(self):
        self.send()
        self.gate.set_publishers([], [])
        self.gate.set_publishers(self.throttle, self.steering)
        self.assertFalse(self.gate.snapshot(10.1)['enabled'])
        self.send(now=10.1)
        self.assertTrue(self.gate.snapshot(10.1)['enabled'])

    def test_controller_restart_changes_writer_identity(self):
        self.send()
        throttle, steering = writers('/controller', b'b')
        self.gate.set_publishers(throttle, steering)
        self.assertFalse(self.gate.snapshot(10.1)['enabled'])
        self.assertFalse(self.gate.receive('throttle', 0.1, b'at', 10.1))

    def test_unchanged_graph_does_not_interrupt_heartbeat(self):
        self.send()
        self.gate.set_publishers(self.throttle, self.steering)
        self.assertTrue(self.gate.snapshot(10.1)['enabled'])

    def test_configured_owner_and_duplicate_node_names(self):
        gate = CommandSafety(required_owner='/desired')
        gate.set_publishers(self.throttle, self.steering)
        self.assertIn('configured command_owner', gate.snapshot(10)['reason'])
        self.gate.set_publishers(self.throttle * 2, self.steering * 2)
        self.assertFalse(self.gate.snapshot(10)['enabled'])

    def test_disconnect_invalidates_pair(self):
        self.send()
        self.gate.invalidate('simulator disconnected')
        self.assertFalse(self.gate.snapshot(10.1)['enabled'])
        self.assertEqual(self.gate.snapshot(10.1)['reason'], 'simulator disconnected')

    def test_invalid_configuration_and_channel(self):
        for timeout in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                CommandSafety(timeout)
        with self.assertRaises(ValueError):
            CommandSafety(required_owner='relative')
        with self.assertRaises(ValueError):
            self.gate.receive('brake', 0.1, b'at', 10)


if __name__ == '__main__':
    unittest.main()
