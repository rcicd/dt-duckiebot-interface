import rospy

from std_srvs.srv import Trigger

from button_driver import ButtonDriver
from duckietown.dtros import HWTest, HWTestJsonParamType


class HWTestButton(HWTest):
    def __init__(self,
                 driver: ButtonDriver,
                 led_blink_secs: int = 2,
                 led_blink_hz: int = 1,
                 ) -> None:
        super().__init__()

        # attr
        self._driver = driver
        self._button_released = False

        # test settings
        self.led_blink_secs = led_blink_secs
        self.led_blink_hz = led_blink_hz

        # test services
        self._desc_tst_srv = rospy.Service(f"~test/desc", Trigger, self.srv_cb_tst_desc)
        self._tst_srv = rospy.Service(f"~test/run", Trigger, self._tst)

    def test_id(self) -> str:
        return "Top button"

    def test_desc_preparation(self) -> str:
        return self.html_util_ul([
            "Put your Duckiebot in the ordinary orientation, and make sure you can see and press the top button."
        ])

    def test_desc_expectation(self) -> str:
        return self.html_util_ul([
            f"The top button's LED should start blinking at {self.led_blink_hz} HZ.",
            f"In about {self.led_blink_secs} seconds, it should stop blinking.",
            "After the LED stops blinking, as soon as you press and release the button, the test should finish.",
        ])
    
    def test_desc_log_gather(self) -> str:
        return self.html_util_ul([
            "On your laptop, run the following command to save the logs.",
            "Replace the <code>[path/to/save]</code> to the directory path where you would like to save the logs.",
            "<code>docker -H [your_Duckiebot_hostname].local logs duckiebot-interface > [path/to/save/]logs-db-iface.txt</code>",
        ])

    def button_event_cb(self):
        self._button_released = True
        
    def _tst(self, _):
        rospy.loginfo(f"[{self.test_id()}] Test service called.")
        success = True

        try:
            # button led test
            self._driver.led.blink_led(
                secs_to_blink=self.led_blink_secs,
                blink_freq_hz=self.led_blink_hz,
            )
            # button press event test
            self._driver.start_test(self.button_event_cb)
            while not self._button_released:
                rospy.sleep(0.1)
            # reset
            self._button_released = False
        except Exception as e:
            rospy.logerr(f"[{self.test_id()}] Experienced error: {e}")
            success = False

        params = f"[{self.test_id()}] led_blink_secs = {self.led_blink_secs}, led_blink_hz = {self.led_blink_hz}"

        return self.format_response_object(
            success=success,
            lst_blocks=[
                self.format_obj(
                    key="Test parameters",
                    value_type=HWTestJsonParamType.STRING,
                    value=params,
                ),
            ],
        )
