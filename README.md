# Robot Grasping with FoundationPose Foundation Model

Project Overview:
FoundationPose is a SOTA foundation model approach to 6D pose estimation and 3D object tracking. How can we use this to perform precise robotic grasping?

Approach: Run 2x instances of FoundationPose - 1x on robot hand/configuration, 1x on object you want to grasp
- This should give you the translation (tx, ty, tz) and rotation (r11, r12, r13, r21, r22, r23, r31, r32, r33) of robot hand/configuration-to-camera and object-to-camera.
- With these two, you should be able to calculate the hand/configuration-to-object pose matrix, if this matrix is accurate enough you should be able to grasp objects.
- The end goal is to be able to run foundationpose on ANY object and ANY robot and have it work.

- Preliminary experiments will be done with the HOPE dataset and Panda Frank Emika Robot



Setup/Environment + How to Run demo + Pre-trained model

Please follow the environment setup and demo instructions from the FoundationPose github repository.

![image](https://github.com/user-attachments/assets/7b36bcb5-80ff-49f6-85f6-73eb2e48a1fc)

![image](https://github.com/user-attachments/assets/7effbfb0-689c-4f1d-a247-9622b9d41401)




Acknowledgements
This project is based off of FoundationPose ofcourse: https://github.com/NVlabs/FoundationPose?tab=readme-ov-file
