import signal
import sys
import threading

import rospy

from python_qt_binding.QtWidgets import QApplication
import jointInfo
import amazing_gui




if __name__ == '__main__':
    
    try:
        rospy.init_node('command_gui')
        app = QApplication(sys.argv)
        app.setApplicationDisplayName("Command Publisher")
        num_rows = jointInfo.get_param('num_rows', 0)
        jsp_gui = amazing_gui.JointStatePublisherGui("Node: " + rospy.get_name(),
                                                                jointInfo.JointInfo(),
                                                                num_rows)
        jsp_gui.show()
        jsp_gui.sliderUpdateTrigger.emit()

        threading.Thread(target=jsp_gui.jsp.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sys.exit(app.exec_())
        
        

    except rospy.ROSInterruptException:
        pass
        
    except Exception as e:
        print(e)

