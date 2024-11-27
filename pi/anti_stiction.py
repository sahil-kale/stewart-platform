import numpy as np


class AntiStictionController:
    """
    This class implements coloumbic and viscous friction compensation. It works by firing a pulse of a given amplitude, and decaying to a static compensation value over a given time.
    """

    # define an enum for the state of the anti-stiction controller
    class State:
        STATIC = 0
        FIRED = 1

    def __init__(
        self,
        firing_threshold,
        firing_amplitude,
        damping_feedforward,
        decay_time_s,
    ):
        self.firing_threshold = firing_threshold
        self.firing_amplitude = firing_amplitude
        self.decay_time_s = decay_time_s
        self.damping_feedforward = damping_feedforward
        self.last_time = None

        self.state = self.State.STATIC
        self.firing_time = None

    def run(self, controller_output, time):
        controller_output_sign = np.sign(controller_output)
        anti_stiction_feedforward = 0

        if self.state == self.State.STATIC:
            if np.abs(controller_output) > self.firing_threshold:
                self.state = self.State.FIRED
                self.firing_time = time
                anti_stiction_feedforward = (
                    self.firing_amplitude * controller_output_sign
                )
        elif self.state == self.State.FIRED:
            if np.abs(controller_output) < self.firing_threshold:
                self.state = self.State.STATIC
            else:
                # calculate the time since the firing
                time_since_firing = time - self.firing_time
                # clamp min time since firing to 0
                time_since_firing = max(time_since_firing, 0)

                # interpolate between the firing amplitude and the damping feedforward as a function of the decay time
                anti_stiction_feedforward = (
                    np.interp(
                        time_since_firing,
                        [0, self.decay_time_s],
                        [self.firing_amplitude, self.damping_feedforward],
                    )
                    * controller_output_sign
                )

        return anti_stiction_feedforward
