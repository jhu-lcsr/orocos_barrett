import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette,QColor
from python_qt_binding.QtCore import Qt
import random

import std_msgs.msg

class BarrettDashboard(Plugin):

    def __init__(self, context):
        super(BarrettDashboard, self).__init__(context)

        self._dotcode_sub = None

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

        jp_widgets = [getattr(self._widget,'jp_%d' % i) for i in range(7)]
        jn_widgets = [getattr(self._widget,'jn_%d' % i) for i in range(7)]
        self.joint_widgets = zip(jp_widgets,jn_widgets)

        tp_widgets = [getattr(self._widget,'tp_%d' % i) for i in range(7)]
        tn_widgets = [getattr(self._widget,'tn_%d' % i) for i in range(7)]
        self.torque_widgets = zip(tp_widgets,tn_widgets)

        for (tp,tn) in self.torque_widgets:
            tp.setRange(0.0,1.0,False)
            tn.setRange(1.0,0.0,False)
            # set random values for testing
            v = (2.0*random.random()) - 1.0
            tp.setValue(v if v >=0 else 0)
            tn.setValue(-v if v <0 else 0)

        for (jp,jn) in self.joint_widgets:
            jp.setRange(0.0,1.0,False)
            jn.setRange(1.0,0.0,False)
            # set random values for testing
            v = (2.0*random.random()) - 1.0
            jp.setValue(v if v >=0 else 0)
            jn.setValue(-v if v <0 else 0)

        background_color = Qt.black
        joint_fill_color = QColor(80,148,204)
        torque_fill_color = QColor(87,186,142)
        joint_alarm_color = QColor(232,97,97)
        torque_alarm_color = QColor(255,103,43)

        for w in jp_widgets + jn_widgets:
            w.setAlarmLevel(0.66)
            w.setFillColor(joint_fill_color)
            w.setAlarmColor(joint_alarm_color)
            p = w.palette()
            p.setColor(tp.backgroundRole(), p.mid().color())
            w.setPalette(p)

        for w in tp_widgets + tn_widgets:
            w.setAlarmLevel(0.66)
            w.setFillColor(torque_fill_color)
            w.setAlarmColor(torque_alarm_color)
            p = w.palette()
            p.setColor(tp.backgroundRole(), p.mid().color())
            w.setPalette(p)

        #self._widget.subscribe_button.clicked[bool].connect(self._handle_subscribe_clicked)

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

    def _handle_subscribe_clicked(self, checked):
        if checked:
            topic_name = self._widget.topic_name.text()
            if len(topic_name) > 0:
                self._dotcode_sub = rospy.Subscriber(
                        topic_name,
                        std_msgs.msg.String,
                        self._dotcode_msg_cb)
            else:
                return False
        else:
            if self._dotcode_sub:
                self._dotcode_sub.unregister()
                self._dotcode_sub = None

    def _dotcode_msg_cb(self, msg):
        self._widget.xdot_widget.set_dotcode(msg.data)
    
