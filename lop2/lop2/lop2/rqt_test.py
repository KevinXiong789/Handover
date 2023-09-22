import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import rclpy
from rqt_gui_py.plugin import Plugin


class MyApplication(QMainWindow):
    def __init__(self, rviz_plugin):
        super(MyApplication, self).__init__()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        layout = QVBoxLayout(self.central_widget)

        # Initialize RViz2 Plugin and add to the layout
        rviz_plugin.create_widget()  # This might be different based on the RViz2 plugin's implementation
        layout.addWidget(rviz_plugin.widget)




# add main function

def main():
    # Initialize the application
    app = QApplication(sys.argv)
    rclpy.init(args=sys.argv)

    # Initialize the RViz2 Plugin
    rviz_plugin = Plugin()  # You need to get the correct plugin instance for RViz2

    window = MyApplication(rviz_plugin)
    window.show()

    sys.exit(app.exec_())
