import Tkinter as tk
import tkMessageBox
import subprocess
import rospy

class PickPoseGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pick Pose GUI")
        
        # Set default values
        self.target_frame = "target_frame"
        self.parent_frame = "world"
        self.approach_distance = 0.2
        self.lift_distance = 0.1
        
        # Create main frame
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(pady=20)
        
        # Title Label
        self.title_label = tk.Label(self.main_frame, text="Pick Pose", font=("Helvetica", 16))
        self.title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Target Frame Label and Field
        self.target_frame_label = tk.Label(self.main_frame, text="Target Frame:")
        self.target_frame_label.grid(row=1, column=0, sticky="e", padx=10)
        
        self.target_frame_entry = tk.Entry(self.main_frame)
        self.target_frame_entry.insert(0, self.target_frame)  # Set default value
        self.target_frame_entry.grid(row=1, column=1, padx=10)
        
        # Parent Frame Label and Field
        self.parent_frame_label = tk.Label(self.main_frame, text="Parent Frame:")
        self.parent_frame_label.grid(row=2, column=0, sticky="e", padx=10)
        
        self.parent_frame_entry = tk.Entry(self.main_frame)
        self.parent_frame_entry.insert(0, self.parent_frame)  # Set default value
        self.parent_frame_entry.grid(row=2, column=1, padx=10)

        # Approach Distance Label and Field
        self.approach_distance_label = tk.Label(self.main_frame, text="Approach Distance (m):")
        self.approach_distance_label.grid(row=3, column=0, sticky="e", padx=10)
        
        self.approach_distance_entry = tk.Entry(self.main_frame)
        self.approach_distance_entry.insert(0, str(self.approach_distance))  # Set default value
        self.approach_distance_entry.grid(row=3, column=1, padx=10)
        
        # Lift Distance Label and Field
        self.lift_distance_label = tk.Label(self.main_frame, text="Lift Distance (m):")
        self.lift_distance_label.grid(row=4, column=0, sticky="e", padx=10)
        
        self.lift_distance_entry = tk.Entry(self.main_frame)
        self.lift_distance_entry.insert(0, str(self.lift_distance))  # Set default value
        self.lift_distance_entry.grid(row=4, column=1, padx=10)
        
        # Pick button
        self.pick_button = tk.Button(self.main_frame, text="Pick", command=self.on_pick_button_click)
        self.pick_button.grid(row=5, column=0, columnspan=2, pady=20)

        # Sub-buttons and fields
        self.sub_button_1 = tk.Button(self.main_frame, text="Sub Button 1", command=self.on_sub_button_1_click)
        self.sub_button_1.grid(row=6, column=0, pady=5)

        self.sub_button_2 = tk.Button(self.main_frame, text="Sub Button 2", command=self.on_sub_button_2_click)
        self.sub_button_2.grid(row=6, column=1, pady=5)

    def on_pick_button_click(self):
        """Handle the pick button click."""
        target_frame = self.target_frame_entry.get()
        parent_frame = self.parent_frame_entry.get()
        approach_distance = self.approach_distance_entry.get()
        lift_distance = self.lift_distance_entry.get()

        # Show message with user input
        tkMessageBox.showinfo("Pick Pose", "Picking pose for target frame: {}, approach distance: {}, lift distance: {}".format(target_frame, approach_distance, lift_distance))

        # Run rosrun command with parameters
        self.run_ros_command(target_frame, parent_frame, approach_distance, lift_distance)

    def run_ros_command(self, target_frame, parent_frame, approach_distance, lift_distance):
        """Run the ROS command."""
        try:
            # Create the rosrun command
            command = [
                "rosrun", "arm_control", "pick_place_waypoints.py",
                target_frame, parent_frame, str(approach_distance), str(lift_distance)
            ]
            
            # Run the command
            subprocess.Popen(command)
            rospy.loginfo("Running command: {}".format(' '.join(command)))
        except Exception as e:
            rospy.logerr("Failed to run ROS command: {}".format(e))
    
    def on_sub_button_1_click(self):
        """Handle the sub-button 1 click."""
        tkMessageBox.showinfo("Sub Button 1", "Sub Button 1 clicked!")

    def on_sub_button_2_click(self):
        """Handle the sub-button 2 click."""
        tkMessageBox.showinfo("Sub Button 2", "Sub Button 2 clicked!")

if __name__ == "__main__":
    root = tk.Tk()
    gui = PickPoseGUI(root)
    root.mainloop()
