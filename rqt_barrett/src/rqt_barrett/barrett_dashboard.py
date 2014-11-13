import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget,QPalette,QColor
from python_qt_binding.QtCore import Qt,QTimer,Signal
from python_qt_binding.Qwt.Qwt import QwtThermo

import random
import actionlib

import sensor_msgs.msg

from urdf_parser_py.urdf import URDF
import oro_barrett_msgs.msg

class BarrettDashboard(Plugin):

    def __init__(self, context):
        super(BarrettDashboard, self).__init__(context)

        self._joint_sub = None

        # Give QObjects reasonable names
        self.setObjectName('BarrettDashboard')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'barrett_dashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        self._widget.setObjectName('BarrettDashboardPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self.context = context

        jp_widgets = [getattr(self._widget,'jp_%d' % i) for i in range(7)]
        jn_widgets = [getattr(self._widget,'jn_%d' % i) for i in range(7)]
        self.joint_widgets = zip(jp_widgets,jn_widgets)

        tp_widgets = [getattr(self._widget,'tp_%d' % i) for i in range(7)]
        tn_widgets = [getattr(self._widget,'tn_%d' % i) for i in range(7)]
        self.torque_widgets = zip(tp_widgets,tn_widgets)

        self.joint_signals = []
        self.torque_signals = []

        for (tp,tn) in self.torque_widgets:
            tp.setRange(0.0,1.0,False)
            tn.setRange(1.0,0.0,False)
            tp.setValue(0.0)
            tn.setValue(0.0)
            # set random values for testing
            #v = (2.0*random.random()) - 1.0
            #tp.setValue(v if v >=0 else 0)
            #tn.setValue(-v if v <0 else 0)

        for (jp,jn) in self.joint_widgets:
            jp.setRange(0.0,1.0,False)
            jn.setRange(1.0,0.0,False)
            jp.setValue(0.0)
            jn.setValue(0.0)
            # set random values for testing
            #v = (2.0*random.random()) - 1.0
            #jp.setValue(v if v >=0 else 0)
            #jn.setValue(-v if v <0 else 0)

        self.barrett_green = QColor(87,186,142)
        self.barrett_green_dark = self.barrett_green.darker()
        self.barrett_green_light = self.barrett_green.lighter()
        self.barrett_blue = QColor(80,148,204)
        self.barrett_blue_dark = self.barrett_blue.darker()
        self.barrett_blue_light = self.barrett_blue.lighter()
        self.barrett_red = QColor(232,47,47)
        self.barrett_red_dark = self.barrett_red.darker()
        self.barrett_red_light = self.barrett_red.lighter()
        self.barrett_orange = QColor(255,103,43)
        self.barrett_orange_dark = self.barrett_orange.darker()

        #background_color = p.mid().color()
        joint_bg_color = self.barrett_blue_dark.darker()
        joint_fill_color = self.barrett_blue
        joint_alarm_color = self.barrett_blue #self.barrett_blue_light
        torque_bg_color = self.barrett_green_dark.darker()
        torque_fill_color = self.barrett_green
        torque_alarm_color = self.barrett_orange #self.barrett_green_light
        temp_bg_color = self.barrett_red_dark.darker()
        temp_fill_color = self.barrett_orange
        temp_alarm_color = self.barrett_red

        for w in jp_widgets + jn_widgets:
            w.setAlarmLevel(0.80)
            w.setFillColor(joint_fill_color)
            w.setAlarmColor(joint_alarm_color)
            p = w.palette()
            p.setColor(tp.backgroundRole(), joint_bg_color)
            w.setPalette(p)

        for w in tp_widgets + tn_widgets:
            w.setAlarmLevel(0.66)
            w.setFillColor(torque_fill_color)
            w.setAlarmColor(torque_alarm_color)
            p = w.palette()
            p.setColor(tp.backgroundRole(), torque_bg_color)
            w.setPalette(p)

        self._widget.hand_temp.setAlarmLevel(0.66)
        self._widget.hand_temp.setFillColor(temp_fill_color)
        self._widget.hand_temp.setAlarmColor(temp_alarm_color)
        p = self._widget.hand_temp.palette()
        p.setColor(self._widget.hand_temp.backgroundRole(), temp_bg_color)
        self._widget.hand_temp.setPalette(p)
        self._widget.hand_temp.setOrientation(Qt.Horizontal, QwtThermo.RightScale)

        self._widget.jf_0.setStyleSheet("QLabel { background-color : rgb(%d,%d,%d); color : rgb(%d,%d,%d); }" % (
            joint_bg_color.red(), joint_bg_color.green(), joint_bg_color.blue(), joint_fill_color.red(), joint_fill_color.green(), joint_fill_color.blue()))

        self.urdf = rospy.get_param('robot_description')
        self.robot = URDF()
        self.robot = self.robot.from_xml_string(self.urdf)

        self.pos_norm = [0] * 7
        self.torque_norm = [0] * 7

        self._joint_sub = rospy.Subscriber(
                'joint_states',
                sensor_msgs.msg.JointState,
                self._joint_state_cb)

        self._status_sub = rospy.Subscriber(
                'status',
                oro_barrett_msgs.msg.BarrettStatus,
                self._status_cb)

        self._hand_status_sub = rospy.Subscriber(
                'hand/status',
                oro_barrett_msgs.msg.BHandStatus,
                self._hand_status_cb)

        self._update_status(-1)
        self.safety_mode = -1
        self.run_mode = 0
        self.hand_run_mode = 0
        self.hand_min_temperature = 40.0
        self.hand_max_temperature = 65.0
        self.hand_temperature = 0.0

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(50)
        self.update_timer.timeout.connect(self._update_widget_values)
        self.update_timer.start()


        self.set_home_client = actionlib.SimpleActionClient(
                'wam/set_home_action',
                oro_barrett_msgs.msg.SetHomeAction)
        self.set_mode_client = actionlib.SimpleActionClient(
                'wam/set_mode_action',
                oro_barrett_msgs.msg.SetModeAction)

        self._widget.button_set_home.clicked[bool].connect(self._handle_set_home_clicked)
        self._widget.button_idle_wam.clicked[bool].connect(self._handle_idle_wam_clicked)
        self._widget.button_run_wam.clicked[bool].connect(self._handle_run_wam_clicked)

        self.bhand_init_client = actionlib.SimpleActionClient(
                'hand/initialize_action',
                oro_barrett_msgs.msg.BHandInitAction)
        self.bhand_set_mode_client = actionlib.SimpleActionClient(
                'hand/set_mode_action',
                oro_barrett_msgs.msg.BHandSetModeAction)

        self._widget.button_initialize_hand.clicked[bool].connect(self._handle_initialize_hand_clicked)
        self._widget.button_idle_hand.clicked[bool].connect(self._handle_idle_hand_clicked)
        self._widget.button_run_hand.clicked[bool].connect(self._handle_run_hand_clicked)

        self._widget.button_open_hand.clicked[bool].connect(self._handle_open_hand_clicked)
        self._widget.button_close_hand.clicked[bool].connect(self._handle_close_hand_clicked)

        self._widget.resizeEvent = self._handle_resize

    def _handle_resize(self, event):
        for i,((w1,w2),(w3,w4)) in enumerate(zip(self.joint_widgets, self.torque_widgets)):
            width = 0.8*getattr(self._widget, 'jcc_%d'%i).contentsRect().width()
            w1.setPipeWidth(width)
            w2.setPipeWidth(width)
            w3.setPipeWidth(width)
            w4.setPipeWidth(width)

    def _handle_set_home_clicked(self, checked):
        if checked:
            self.set_home()

    def _update_status(self, status):
        if status == -1:
            self._widget.safety_mode.setText('UNKNOWN SAFETY MODE')
            self._widget.safety_mode.setToolTip('The WAM is in an unknown state. Proceed with caution.')
            color = QColor(200,200,200)
            self._widget.button_set_home.setEnabled(False)
            self._widget.button_idle_wam.setEnabled(False)
            self._widget.button_run_wam.setEnabled(False)
        elif status == 0:
            self._widget.safety_mode.setText('E-STOP')
            self._widget.safety_mode.setToolTip('The WAM is stopped and unpowered. Determine the source of the fault and reset the arm to continue using it.')
            self._widget.button_set_home.setEnabled(False)
            self._widget.button_idle_wam.setEnabled(False)
            self._widget.button_run_wam.setEnabled(False)
            color = self.barrett_red
        else:
            if not self.homed:
                self._widget.safety_mode.setText('UNCALIBRATED')
                self._widget.safety_mode.setToolTip('The WAM is not calibrated. Please place it in the calibration posture and click the "Calibrate" button.')
                self._widget.button_set_home.setEnabled(True)
                self._widget.button_idle_wam.setEnabled(False)
                self._widget.button_run_wam.setEnabled(False)
                color = self.barrett_orange
            else:
                if status == 1:
                    self._widget.safety_mode.setText('IDLE')
                    self._widget.safety_mode.setToolTip('The WAM is running but all joints are passive. It is safe to home the arm.')
                    self._widget.button_set_home.setEnabled(True)
                    self._widget.button_idle_wam.setEnabled(True)
                    self._widget.button_run_wam.setEnabled(True)
                    color = self.barrett_blue
                elif status == 2:
                    self._widget.safety_mode.setText('ACTIVE')
                    self._widget.safety_mode.setToolTip('The WAM is running and all joints are active. Proceed with caution.')
                    self._widget.button_set_home.setEnabled(False)
                    self._widget.button_idle_wam.setEnabled(False)
                    self._widget.button_run_wam.setEnabled(False)
                    color = self.barrett_green

        darker = color.darker()
        lighter = color.lighter()
        self._widget.safety_mode.setStyleSheet("QLabel { background-color : rgb(%d,%d,%d); color : rgb(%d,%d,%d); }" % (
            color.red(), color.green(), color.blue(), lighter.red(), lighter.green(), lighter.blue()))

    def _update_widget_values(self):

        if self.safety_mode in (oro_barrett_msgs.msg.SafetyMode.ACTIVE, oro_barrett_msgs.msg.SafetyMode.IDLE):
            for (v,(tp,tn)) in zip(self.torque_norm,self.torque_widgets):
                tp.setEnabled(True)
                tn.setEnabled(True)
                tp.setValue(v if v >=0 else 0)
                tn.setValue(-v if v <0 else 0)


            for (v,(jp,jn)) in zip(self.pos_norm,self.joint_widgets):
                jp.setEnabled(True)
                jn.setEnabled(True)
                jp.setValue(v if v >=0 else 0)
                jn.setValue(-v if v <0 else 0)
        else:
            for (p,n) in self.joint_widgets + self.torque_widgets:
                p.setEnabled(True)
                n.setEnabled(True)

        self._widget.hand_temp.setValue((self.hand_temperature-self.hand_min_temperature)/(self.hand_max_temperature-self.hand_min_temperature))

        self._update_status(self.safety_mode)
        self._update_buttons(self.run_mode, self.hand_run_mode)

    def _update_buttons(self, run_mode, hand_run_mode):
        if run_mode == oro_barrett_msgs.msg.RunMode.IDLE:
            self._widget.button_idle_wam.setChecked(True)
            self._widget.button_run_wam.setChecked(False)
        else:
            self._widget.button_idle_wam.setChecked(False)
            self._widget.button_run_wam.setChecked(True)

        if hand_run_mode == oro_barrett_msgs.msg.RunMode.RUN:
            self._widget.button_idle_hand.setChecked(False)
            self._widget.button_run_hand.setChecked(True)
        else:
            self._widget.button_idle_hand.setChecked(True)
            self._widget.button_run_hand.setChecked(False)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _joint_state_cb(self, msg):
        joint_pos_map = dict(zip(msg.name, msg.position))

        for (name,pos,torque,i) in zip(msg.name,msg.position,msg.effort,range(7)):
            joint = self.robot.joint_map[name]
            self.pos_norm[i] = 2.0*((pos-joint.limit.lower)/(joint.limit.upper-joint.limit.lower)) - 1.0
            self.torque_norm[i] = torque/joint.limit.effort

    def _status_cb(self, msg):
        self.safety_mode = msg.safety_mode.value
        self.run_mode = msg.run_mode.value
        self.homed = msg.homed

        if self.safety_mode == oro_barrett_msgs.msg.SafetyMode.ACTIVE:
            self._widget.button_initialize_hand.setEnabled(False)
        elif self.safety_mode == oro_barrett_msgs.msg.SafetyMode.ESTOP:
            self._widget.button_initialize_hand.setEnabled(False)
            self._widget.button_idle_hand.setEnabled(False)
            self._widget.button_run_hand.setEnabled(False)

    def _hand_status_cb(self, msg):
        self.hand_initialized = msg.initialized
        self.hand_temperature = max(msg.temperature)
        self.hand_run_mode = msg.run_mode.value
            
    def _handle_set_home_clicked(self, checked):
        goal = oro_barrett_msgs.msg.SetHomeGoal()
        self.set_home_client.send_goal(goal)

    def _handle_idle_wam_clicked(self, checked):
        if checked:
            goal = oro_barrett_msgs.msg.SetModeGoal()
            goal.mode.value = oro_barrett_msgs.msg.RunMode.IDLE
            self.set_mode_client.send_goal(goal)

    def _handle_run_wam_clicked(self, checked):
        if checked:
            goal = oro_barrett_msgs.msg.SetModeGoal()
            goal.mode.value = oro_barrett_msgs.msg.RunMode.RUN
            self.set_mode_client.send_goal(goal)

    def _handle_initialize_hand_clicked(self, checked):
        if self.safety_mode ==  oro_barrett_msgs.msg.SafetyMode.IDLE:
            goal = oro_barrett_msgs.msg.BHandInitGoal()
            self.bhand_init_client.send_goal(goal)

    def _handle_run_hand_clicked(self, checked):
        if checked:
            goal = oro_barrett_msgs.msg.BHandSetModeGoal()
            goal.run_mode.value = oro_barrett_msgs.msg.RunMode.RUN
            self.bhand_set_mode_client.send_goal(goal)

    def _handle_idle_hand_clicked(self, checked):
        if checked:
            goal = oro_barrett_msgs.msg.BHandSetModeGoal()
            goal.run_mode.value = oro_barrett_msgs.msg.RunMode.IDLE
            self.bhand_set_mode_client.send_goal(goal)

    def _handle_open_hand_clicked(self, checked):
        pass

    def _handle_close_hand_clicked(self, checked):
        pass
        
