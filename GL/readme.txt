COMP 5812M Assignment 1
Foundations of Modelling and rendering
FakeGL

------------------------------------------
usage:

To compile on feng-linux / feng-gps:

module add qt/5.13.0
qmake -project QT+=opengl
qmake
make

To run on feng-linux / feng-gps:
./FakeGLRenderWindowRelease ../path_to/model.obj ../path_to/texture.ppm

------------------------------------------
In this assignment, we implement the OpenGL pipeline in C++.

I implement all function, but sadly there are still some bugs in the program. such as Modulation function: if we turn on lots of other function, sometimes it shows nothing , but most of time it can really work(so please reopen it and try it again if it doesn't work).
But now,need to focus on other assigments. maybe I'll try to fix them if I have time.

Overall, it's a good experience for me, it really helps me learn much about how OpenGL works.
More thoughts are in program comment.

Got great help from:
(https://learnopengl.com/)
(http://www.songho.ca/opengl/index.html)
------------------------------------------



 

