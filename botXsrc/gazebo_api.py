from botX.components import BaseComponent
from botX.applicatons import external_command_pool
import rospy

class GazeboAPI(BaseComponent):

    def setup(self):
        """
        download .gazebo from github url and put it in absolute path
        """
        command = 'roslaunch haptica_gazebo jaco_qr_demo.launch'
        self.proc_id = external_command_pool.start_command(command)
        """
        wait until it is lauched
        """
        self.buf = []
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber("chatter", String, self.cache_info)
        """
        wait until it is receiving
        """

    def cache_info(self, msg):
        self.buf.push(msg)

    def get_environment_status(self):
        """
        preprocess it to usable format
        """
        return self.buf[-1]

    def shutdown(self):
        external_command_pool.end_command(self.proc_id)
