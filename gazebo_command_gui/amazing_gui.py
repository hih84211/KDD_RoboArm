import math
import random

import rospy
from std_msgs.msg import Float64

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget

RANGE = 10000


class JointStatePublisherGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self, title, jsp, num_rows=0):
        super(JointStatePublisherGui, self).__init__()
        self.setWindowTitle(title)
        self.jsp = jsp
        self.joint_map = {}
        self.vlayout = QVBoxLayout(self)
        self.scrollable = QWidget()
        self.gridlayout = QGridLayout()
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        
        self.jsp.set_source_update_cb(self.source_update_cb)

        font = QFont("Helvetica", 9, QFont.Bold)

        ### Generate sliders ###
        sliders = []
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]
            if joint['min'] == joint['max']:
                continue

            joint_layout = QVBoxLayout()
            row_layout = QHBoxLayout()

            label = QLabel(name)
            label.setFont(font)
            row_layout.addWidget(label)
            display = QLineEdit("0.00")
            display.setAlignment(Qt.AlignRight)
            display.setFont(font)
            display.setReadOnly(True)
            row_layout.addWidget(display)

            joint_layout.addLayout(row_layout)

            slider = QSlider(Qt.Horizontal)

            slider.setFont(font)
            slider.setRange(0, RANGE)
            slider.setValue(RANGE/2)

            joint_layout.addWidget(slider)

            self.joint_map[name] = {'slidervalue': 0, 'display': display,
                                    'slider': slider, 'joint': joint}
            # Connect to the signal provided by QSignal
            slider.valueChanged.connect(lambda event,name=name: self.onValueChangedOne(name))

            sliders.append(joint_layout)
        
        
        self.publishers =dict(zip(self.jsp.joint_list, [rospy.Publisher('/cnc_test17/x_position_controller/command', Float64, queue_size = 1), 
                                                        rospy.Publisher('/cnc_test17/y_position_controller/command', Float64, queue_size = 1), 
                                                        rospy.Publisher('/cnc_test17/z_position_controller/command', Float64, queue_size = 1),
                                                        rospy.Publisher('/cnc_test17/nozzlebase_controller/command', Float64, queue_size = 1),
                                                        rospy.Publisher('/cnc_test17/nozzle_controller/command', Float64, queue_size = 1)])) 

        # Determine number of rows to be used in grid
        self.num_rows = num_rows
        # if desired num of rows wasn't set, default behaviour is a vertical layout
        if self.num_rows == 0:
            self.num_rows = len(sliders)  # equals VBoxLayout
        # Generate positions in grid and place sliders there
        self.positions = self.generate_grid_positions(len(sliders), self.num_rows)
        for item, pos in zip(sliders, self.positions):
            self.gridlayout.addLayout(item, *pos)

        # Set zero positions read from parameters
        self.center()

        # Synchronize slider and displayed value
        self.sliderUpdate(None)

        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        self.scrollable.setLayout(self.gridlayout)
        self.scroll.setWidget(self.scrollable)
        self.vlayout.addWidget(self.scroll)

        # Buttons for randomizing and centering sliders and
        # Spinbox for on-the-fly selecting number of rows
        self.randbutton = QPushButton('Randomize', self)
        self.randbutton.clicked.connect(self.randomize_event)
        self.vlayout.addWidget(self.randbutton)
        self.ctrbutton = QPushButton('Center', self)
        self.ctrbutton.clicked.connect(self.center_event)
        self.vlayout.addWidget(self.ctrbutton)
        self.maxrowsupdown = QSpinBox()
        self.maxrowsupdown.setMinimum(1)
        self.maxrowsupdown.setMaximum(len(sliders))
        self.maxrowsupdown.setValue(self.num_rows)
        self.maxrowsupdown.valueChanged.connect(self.reorggrid_event)
        self.vlayout.addWidget(self.maxrowsupdown)
        self.setLayout(self.vlayout)
        

    def source_update_cb(self):
        self.sliderUpdateTrigger.emit()

    def onValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        joint_info['slidervalue'] = joint_info['slider'].value()
        joint = joint_info['joint']
        joint['position'] = self.sliderToValue(joint_info['slidervalue'], joint)
        joint_info['display'].setText("%.2f" % joint['position'])
        self.publishers[name].publish(joint['position'])

    @pyqtSlot()
    def updateSliders(self):
        self.update_sliders()

    def update_sliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].setValue(joint_info['slidervalue'])
            

    def center_event(self, event):
        self.center()

    def center(self):
        rospy.loginfo("Centering")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(self.valueToSlider(joint['zero'], joint))

    def reorggrid_event(self, event):
        self.reorganize_grid(event)

    def reorganize_grid(self, number_of_rows):
        self.num_rows = number_of_rows

        # Remove items from layout (won't destroy them!)
        items = []
        for pos in self.positions:
            item = self.gridlayout.itemAtPosition(*pos)
            items.append(item)
            self.gridlayout.removeItem(item)

        # Generate new positions for sliders and place them in their new spots
        self.positions = self.generate_grid_positions(len(items), self.num_rows)
        for item, pos in zip(items, self.positions):
            self.gridlayout.addLayout(item, *pos)

    def generate_grid_positions(self, num_items, num_rows):
        if num_rows == 0:
            return []
        positions = [(y, x) for x in range(int((math.ceil(float(num_items) / num_rows)))) for y in range(num_rows)]
        positions = positions[:num_items]
        return positions

    def randomize_event(self, event):
        self.randomize()

    def randomize(self):
        rospy.loginfo("Randomizing")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(
                    self.valueToSlider(random.uniform(joint['min'], joint['max']), joint))

    def sliderUpdate(self, event):
        for name, joint_info in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].value()
        self.update_sliders()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue
