
import rospy

from oro_barrett_interface.grasp_action import GraspAction


class BHandActionServer(object):

    def __init__(self, name="bhand_action_server"):
        self.actions = {}

        # Add grasp action
        grasp = GraspAction(name='grasp', parent=self)
        self.actions[grasp.name] = grasp

    def preempt_peers_of(self, name):
        """Preempt all peers of the given action. This enables coupling of
        multiple action servers."""
        for peer_name, action in self.actions.items():
            if peer_name != name:
                action.peer_preempt_cb()


def main():
    rospy.init_node('bhand_action_server')

    BHandActionServer()

    rospy.spin()

if __name__ == '__main__':
    main()
