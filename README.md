# RigidBodyDemo
A very simple rigid body simulation with Qt in C++

Please check out the YouTube video in here to see what it looks like:
https://youtu.be/5Rg6wxWv9X4

Here's a video transcript with added more details:

0:00) This is my demo of a rigid body simulation.

0:04) In this 3D view drawn with OpenGL, you can see six ridid bodies (rectangles) connected each other with blue dots that are called "body springs". 

0:10) Each pair of dots are represented a spring which stretches/shrinks in this space based on the physics law.

0:14) They are also connected to the 3D space with purple dots that are called "world springs."

0:19) With this "parameters" text file, the cordinates of bodies are described.
The "parameters" text file can be selected and read with the file browser, so you can try with a different settings in the file.

0:37) There are a few widgets you can adjust how interact those bodies in real-time.

0:42) Wind slider calls wind.

0:48)  A button to disable body springs. Without the body springs, they are moving freely from each other.

0:58) Another button to cut world springs. With this button, those bodies start falling. 

1:02) Click the button again so they are connected to the world springs and bouncing back.

1:09) Finally they are connected again each other.

1:13) So you can use this for instance, a rag doll type of physics.

1:23) Thank you for watching.

To build) this project has been made with a very old Xcode.
So the project files are provided as a zip file, as well as ".plist" and ".pro" files.
Extract the zip file and see if you can open the project with a extracted ".xcodeproj" folder.
