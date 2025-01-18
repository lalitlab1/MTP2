import Tkinter as tk
import tkMessageBox
import subprocess
import rospy
import json
import tf2_ros


class PickPoseGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pick Pose GUI")

        # Mode: Pick or Place (default: Pick)
        self.mode = tk.StringVar(value="pick")
        
        # Default frames for Pick & Place
        self.frames = {
            "pick": {"parent": "marker_pallet_base", "child": "target_pick"},
            "place": {"parent": "marker_station", "child": "target_place"}
        }

        # Main frame
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(pady=20)

        # Mode switch: Pick / Place
        self.mode_label = tk.Label(self.main_frame, text="Mode:")
        self.mode_label.grid(row=0, column=0, padx=10, sticky="e")

        self.mode_switch = tk.OptionMenu(self.main_frame, self.mode, "pick", "place", command=self.update_frames)
        self.mode_switch.grid(row=0, column=1, padx=10)

        # Parent Frame
        self.parent_frame_label = tk.Label(self.main_frame, text="Parent Frame:")
        self.parent_frame_label.grid(row=1, column=0, sticky="e", padx=10)

        self.parent_frame_entry = tk.Entry(self.main_frame)
        self.parent_frame_entry.grid(row=1, column=1, padx=10)

        # Child Frame
        self.child_frame_label = tk.Label(self.main_frame, text="Child Frame:")
        self.child_frame_label.grid(row=2, column=0, sticky="e", padx=10)

        self.child_frame_entry = tk.Entry(self.main_frame)
        self.child_frame_entry.grid(row=2, column=1, padx=10)

        # Pick button
        self.pick_button = tk.Button(self.main_frame, text="Pick", command=self.on_pick_button_click)
        self.pick_button.grid(row=3, column=0, columnspan=2, pady=10)

        # Calibration buttons
        self.pick_calibrate_button = tk.Button(self.main_frame, text="Pick Calibration", command=lambda: self.calibrate("pick_calibration.json"))
        self.pick_calibrate_button.grid(row=4, column=0, pady=5)

        self.place_calibrate_button = tk.Button(self.main_frame, text="Place Calibration", command=lambda: self.calibrate("place_calibration.json"))
        self.place_calibrate_button.grid(row=4, column=1, pady=5)

        # Set default frames
        self.update_frames()

    def update_frames(self, *_):
        """Update parent and child frames based on selected mode."""
        mode = self.mode.get()
        self.parent_frame_entry.delete(0, tk.END)
        self.child_frame_entry.delete(0, tk.END)
        self.parent_frame_entry.insert(0, self.frames[mode]["parent"])
        self.child_frame_entry.insert(0, self.frames[mode]["child"])

    def on_pick_button_click(self):
        """Handle pick button click."""
        parent_frame = self.parent_frame_entry.get()
        child_frame = self.child_frame_entry.get()

        tkMessageBox.showinfo("Pick Pose", "Picking pose for parent: {}, child: {}".format(parent_frame, child_frame))

    def calibrate(self, filename):
        """Get transform between frames and save to JSON."""
        rospy.init_node("calibration_node", anonymous=True)
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        parent_frame = self.parent_frame_entry.get()
        child_frame = self.child_frame_entry.get()

        try:
            rospy.loginfo("Looking up transform from {} to {}".format(parent_frame, child_frame))
            transform = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(3.0))

            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            # Prepare JSON format with "transforms" key
            transform_data = {
                "transforms": [
                    {
                        "parent": parent_frame,
                        "child": child_frame,
                        "translation": [translation.x, translation.y, translation.z],
                        "rotation": [rotation.x, rotation.y, rotation.z, rotation.w]
                    }
                ]
            }

            # Save to JSON file
            with open(filename, "w") as f:
                json.dump(transform_data, f, indent=4)

            tkMessageBox.showinfo("Calibration Saved", "Transform saved to {}".format(filename))
            rospy.loginfo("Transform saved: {}".format(transform_data))

        except Exception as e:
            tkMessageBox.showerror("Error", "Failed to get transform: {}".format(e))
            rospy.logerr("Failed to get transform: {}".format(e))


if __name__ == "__main__":
    root = tk.Tk()
    gui = PickPoseGUI(root)
    root.mainloop()
