#
# Mars Rover Design Team
# idle.py
#
# Created on Nov 20, 2020
# Updated on Aug 21, 2022
#

import core
import time
import asyncio
import random
import logging
import interfaces
from core.states import RoverState


class Stuck(RoverState):
    """
    In this state the program will command the rover to stop as it is stuck.
    Its singular purpose is to prevent the rover from killing itself into a marker or obstacle.
    """

    def start(self):
        # Intialize state member variables.
        self.logger = logging.getLogger(__name__)
        # Stop drive.
        interfaces.drive_board.stop()
        # Set random seed.
        random.seed(time.time())
        self.rand_num = random.randint(1, 10)

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events

        :param event:
        :return: RoverState
        """
        state: RoverState = None

        if event == core.AutonomyEvents.START:
            # Change states to Idle
            state = core.states.Idle()

        elif event == core.AutonomyEvents.ABORT:
            # Change states to Idle
            state = core.states.Idle()

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = self

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state

    async def run(self):
        """
        Defines regular rover operation when under this state
        """
        # Print log instructions.
        if self.rand_num == 1:
            self.logger.warning(
                "Well, doggone it! That there Rover fella done figured out he's got himself all tangled up like a ticklin' tumbleweed in a twister. Now, y'all got two options to get him outta this predicament: Give that 'START AUTONOMY' button a poke or press the 'STOP AUTONOMY' button to fetch him back to his lazybones state. Either way, Rover's hopin' y'all can lend him a paw, 'cause this situation is stickier than molasses on a summer porch swing!"
            )
        elif self.rand_num == 2:
            self.logger.warning(
                "Well, dagnabbit! This here rover done gone and figured out it's plum stuck. Y'all got two options: Either give 'er a good ol' holler with the START AUTONOMY hootenanny or holler STOP AUTONOMY to reckon it back to its idle state. Remember, both them buttons do the trick, so pick yer poison and let the shenanigans begin!"
            )
        elif self.rand_num == 3:
            self.logger.warning(
                "Well, shucks! That fancy-pants rover done gone and reckoned it's plum stuck in a pickle jar! Y'all got two options now: hit that snazzy lil' START AUTONOMY button to let 'er rest easy, or give 'er a holler with the STOP AUTONOMY button to also bring 'er back to the snoozeville state. Git ready for a wild ride, y'all!"
            )
        elif self.rand_num == 4:
            self.logger.warning(
                "Well, dadgummit! That there fancy Rover contraption done gone and decided it's plumb stuck! Ya reckon we oughta give 'er a good ol' hickifyin' treatment? Y'all got two options: hit that there START AUTONOMY button and watch 'er kick back into idle, or give 'er a holler with the STOP AUTONOMY button to also bring 'er on back to the lazy daisy idle state. Yeehaw!"
            )
        elif self.rand_num == 5:
            self.logger.warning(
                "Well, shoot dang! That fancy ol' Rover contraption done figured out it's in a mighty pickle. So, reckon ya got yerself two options: reckon ya can press the dandy ol' START AUTONOMY button or the STOP AUTONOMY button to get that thingamajig back to its good ol' idle state. Either way, that Rover gal gonna be sittin' pretty, just chillin' like a cucumber on a hot summer day."
            )
        elif self.rand_num == 6:
            self.logger.warning(
                "Like, oh my gosh! The rover's all like, 'I'm totally stuck, you guys!' So, here's the deal: you gotta, like, choose between pressing the fabulously hip START AUTONOMY or the totally groovy STOP AUTONOMY buttons to get it back to its chill idle state. No matter which button you pick, this rover babe's gonna be all chillaxing and having a blast, ya know? So, let's give it some love and get it out of its sticky situation, alright? Toodles!"
            )
        elif self.rand_num == 7:
            self.logger.warning(
                "Oh my gosh, like, the rover is, like, totally over it and is, like, convinced that it's, like, stuck and stuff. So, like, you have the option to, like, totally press the START AUTONOMY or STOP AUTONOMY buttons, and either way, she's just gonna be, like, super chill and go back to, like, her idle state, you know? It's, like, no biggie, she's just, like, chilling and waiting for some cosmic inspiration or whatever. So, like, do your thing and let the rover, like, groove in her own laid-back way, okay?"
            )
        elif self.rand_num == 8:
            self.logger.warning(
                "Yo, dude! The rover's like, 'Bro, I'm totally stuck, man.' So, check this out: you gotta hit either the gnarly 'START AUTONOMY' button or the chillin' 'STOP AUTONOMY' button to bring it back to the idle state. But hey, no worries, dude! Whether you go full autonomous or stop the autonomy, that rover's just kickin' it and having a chill time. Stay cool, bro! 🤙"
            )
        elif self.rand_num == 9:
            self.logger.warning(
                "Dude, the Rover's like, 'Whoa, man, I'm totally stuck, bro.' But don't worry, there's a chill solution! Check it out, we got two gnarly buttons: 'START AUTONOMY' and 'STOP AUTONOMY.' No matter which one you hit, this Rover's just gonna kick back and chillax in the idle state, dude. So, like, choose your own adventure, man. Keep the autonomy flowin' or bring it back to chill mode. Either way, this Rover's just cruisin' and enjoyin' the cosmic vibes, man."
            )
        elif self.rand_num == 10:
            self.logger.warning(
                "🤖🚀 Rover: '🆘 I'm 🚫🚶 stuck. ⏯️' 👉 Press ▶️ to activate 🤖🔀 or ❌ to deactivate 🤖🔀 and return to 😴 state. 🤖🚀 Rover: 'Either way, I'm 😎 chillin'!'"
            )

        # Do nothing.
        await asyncio.sleep(core.constants.EVENT_LOOP_DELAY)

        return self
