# Robotics---Motion-Planning-in-CSpace

<a name="br1"></a>CS 4610/5335: Robotic Science and Systems (Spring 2023) Robert Platt
Northeastern University

HW 2: Motion Planning in CSpace

Please remember the following policies:

• Submissions should be made electronically via the Canvas. Please ensure that your solutions for both the

written and/or programming parts are present and zipped into a single ﬁle.

• Solutions may be handwritten or typeset. For the former, please ensure handwriting is legible.

• You are welcome to discuss the programming questions (but not the written questions) with other students in
 the class. However, you must understand and write all code yourself. Also, you must list all students (if any)
 with whom you discussed your solutions to the programming questions.

We recommend that you ﬁrst familiarize yourself with MATLAB and the Robotics Toolbox by fol-
lowing these steps:

• Install MATLAB R2020b. Visit the following article for Northeastern-speciﬁc instructions:

<https://service.northeastern.edu/tech?id=kb_article&sys_id=68c93fd6dbf37bc0c5575e38dc961918>

– You may have to log in (top right) and re-enter the URL to view the page’s content.

– If you have an older version of MATLAB, we recommend installing the R2019b version to ensure better

support from the teaching staﬀ.

– If you have suﬃcient space on your computer, we recommend installing all of the toolboxes as well. At
 a minimum, you should install the Robotics System Toolbox, the toolboxes listed on the following page,
 and the Optimization and Symbolic Math toolboxes.

[https://www.mathworks.com/support/requirements/robotics-system-toolbox.html ](https://www.mathworks.com/support/requirements/robotics-system-toolbox.html)You can always choose to install additional toolboxes later if you are uncertain.

• If you need an introduction or refresher on MATLAB, see the “MATLAB Resources” section of the “Resources”

page on Piazza for slides and practice questions (with answers) from a short but intensive introductory course.

• Install Peter Corke’s Robotics Toolbox (version 10.4). This is diﬀerent from the oﬃcial MATLAB Robotics
 System Toolbox installed in the previous step.

<http://petercorke.com/wordpress/toolboxes/robotics-toolbox>

– The simplest way to install this is from the .mltbx ﬁle:

<http://petercorke.com/wordpress/?ddownload=778>

To install, open the downloaded ﬁle within MATLAB. Then run rtbdemo to check that it is working.

– Version 10.x of the toolbox is the only version compatible with the second edition of the Robotics, Vision

and Control textbook, which is the version we are using.

– You may also want to download the toolbox manual as a code reference:

<http://petercorke.com/wordpress/?ddownload=343>

• You will inevitably encounter errors. Here are some debugging tips:

– The MATLAB documentation is very extensive and available both online and oﬄine. To look up a function
 within the interactive session, type help <function> (e.g., help SE3). The description often contains
 links to further documentation and related functions.

– Use disp liberally to print out variables and other debugging information.

– If you are used to coding in MATLAB purely with matrices, note that the Robotics Toolbox deﬁnes many
 entities as classes and objects (e.g., SE3, SerialLink, etc.). Know the diﬀerence, and do not fret when
 you encounter type conversion issues – ﬁxing them is similar to debugging dimension mismatch issues.

– Tables 2.1 and 2.2 in the textbook (data type conversions) are extremely useful.

1




<a name="br2"></a>Conﬁguration Space and Motion Planning

C0. 2 points. Consider the diagram above. Two square robots A and B operate in a 2-D workspace. The two
 robots do not rotate, and each moves on a ﬁxed track, so that its center remains on the solid gray line shown
 in the ﬁgure. The robots must move so as not to collide with each other. The diagram is to scale, with each

tick denoting a distance of one unit.

(a) Even though there are two robots both moving in a 2-D workspace, we can still use a 2-D conﬁguration
 space to represent the above system. Specify what the axes of our conﬁguration space correspond to in
 the workspace, what the axis limits are, and what conﬁguration the above diagram depicts.

(b) Draw the conﬁguration space. For each conﬁguration-space obstacle, label the coordinates of the vertices,
 and sketch the workspace conﬁguration corresponding to each. Since the workspace is symmetric, only
 makes sketches for the left side of the workspace.

2




<a name="br3"></a>In the remainder of this section, we will revisit the 2-DOF 2-link rotational planar robot shown above that we
considered in lecture. The arm is attached to a table surface that we are viewing from a top-down perspective.
There are obstacles on the table as shown by the shaded regions. The goal is move the arm from start
conﬁguration (1, purple) to the goal conﬁguration (2, orange), without colliding into any obstacles. Assume  that the illustrated workspace is 10 units wide and 10 units high; the origin of frame {0} is at the bottom-left
corner, with x0 and y0 axes as shown. The ﬁrst link of the arm is attached to the table at (6.4, 2.5), the origin  of frame {1}. Link 2 is attached to link 1 at (2.1, 0) with respect to frame {1}, the origin of frame {2}. The
conﬁguration of the arm is given by q = (q1, q2), where q1 is the angle between x0 and x1, and q2 is the angle between x1 and x2. Both joints may rotate between 0 and 2π radians. For example, the start conﬁguration (1,
purple) is qstart = (0.85, 0.9), and the goal conﬁguration (2, orange) is qgoal = (3.05, 0.05).

Download the starter code from Piazza (hw2.zip). In this ﬁle, you will ﬁnd:

• hw2 cspace.m: Main function. Call hw2 cspace(<questionNum>) with an appropriate question number
 (1–7) to run the code for that question. Do not modify this ﬁle! (Feel free to actually make changes, but
 check that your code runs with an unmodiﬁed copy of hw2 cspace.m.)

• C1.m – C7.m: Code stubs that you will have to ﬁll in for the respective questions.

• q2poly.m: Code stub that you will have to ﬁll in, helper function for various questions.

• plot obstacles.m: Helper function to draw workspace obstacles deﬁned in hw2 cspace.m.

3




<a name="br4"></a>C1: Start conﬁguration. C1: Goal conﬁguration.

C1. 2 points. Plot the robot in the workspace, as shown above. The demonstration code in C1.m shows how to

` `plot the robot at the zero conﬁguration (q = (0, 0)). You will need to make appropriate transformations to
 both links’ polygons and their pivot points (frame origins). Consider ﬁlling in q2poly.m ﬁrst, which is a useful
 helper function for C1 and future questions. If you provide qstart and qgoal as input, you should get the ﬁgures
above.

Useful functions: Read the documentation for polyshape, a MATLAB class for deﬁning 2-D polygons.

C2. 2 points. Convert the problem into conﬁguration space by discretizing the conﬁguration space, and checking
 for collisions at each discrete grid point. Using the speciﬁed grid for each axis given in q grid, compute whether
 the conﬁguration at each point is in collision or not, by intersecting the links’ 2-D polygon with the obstacles’
 2-D polygons. Assume that the robot is never in collision with itself. The resulting matrix should look similar
 to the conﬁguration space diagram shown on the slide.

Useful functions: intersect

Hint: Future questions rely on the output of C2.m, which may take a while to compute. To avoid re-computing
it in future questions, we have provided functionality to save it in your MATLAB workspace, and pass it in on
future calls:

cspace = hw2 cspace(2);
hw2 cspace(3, cspace);

C3: Distance transform from goal conﬁguration. C4: Path from start to goal.

C3. 2 points. Given a speciﬁed goal conﬁguration and the conﬁguration-space grid from C2, compute the distance

transform from the grid point nearest to the goal.

C4. 2 points. Using the distance transform from C3, ﬁnd a path from the speciﬁed start conﬁguration’s closest
 grid point to the goal’s grid point. Descend the distance transform in a greedy fashion, breaking ties with any
 strategy you wish. Diagonal neighbors are allowed.

C5. 1 point. Convert the path in grid point indices, found in C4, into a path in conﬁgurations. Remember to
 include the actual start and goal conﬁgurations. This should trigger a visualization similar to the one shown
 above.

4




<a name="br5"></a>C5: Trajectory from start to goal. C6: Swept-volume collisions along the path.

C6. 2 points. Unfortunately, since collisions have only been checked at discrete grid points, we cannot guarantee

that the segments between those grid points are collision-free. In fact, the trajectory we found in our implemen-
tation of C5 contains three collisions, shown in the right above. These collisions can be detected by considering
the swept volume between two conﬁgurations. The swept volume can be approximated by appropriate convex
hulls of the robot links’ 2-D polygons. Check if any segments of the trajectory you found in C5 are in colli-
sion, plot the violating swept volumes (similar to right diagram above), and return the number of collisions.
Depending on how you found your trajectory, it may not actually have any such swept-volume collisions!
Useful functions: convhull

C7. 1 point. Most of the collisions above were caused by planning a path that was too close to obstacles. One

simple conservative way to avoid such collisions is to pad the obstacles by a small buﬀer zone. Pad the obstacles
in conﬁguration space by one grid cell (including diagonal neighbors), and verify that the resulting trajectory
does not contain any swept-volume collisions.

C7: More conservative trajectory from start to goal, with no swept-volume collisions.

5
