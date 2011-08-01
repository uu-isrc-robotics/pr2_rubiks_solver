****************************
* Kociemba Cube Solver 1.0 *
* - written by -           *
*   - Greg Schmidt -       *
****************************

Why write another cube solver that implements Kociemba's
algorithm?  Well I wanted to develop a more complete
understanding of the algorithm and, as Mike Reid suggested,
one way to do that is to write a program to implement the
algorithm.  So this program was written primarily as a
learning exercise.  It offers a simple console interface and
has no fancy graphics although I have tried to make cube
entry simple.  However, if you are more interested in the
the algorithm itself, it is my hope that you will find this
program useful as I have tried to document it well enough so
that others may use this program as a reference for under-
standing how the algorithm works.  If you are completely
unfamiliar with Kociemba's algorithm, you may first want to
browse through the cube.lovers archive at
ftp://ftp.ai.mit.edu/pub/cube-lovers as there are several
messages there describing the basic principles.

+-------------------------------------------------------+
|  Please see cubemain.cpp for instructions on how to   |
|  invoke this program.  Note that the very first time  |
|  this program is run it will generate several files   |
|  containing the move mapping and the pruning tables   |
|  and this will take several minutes to complete.      |
+-------------------------------------------------------+

Files
-----

kcube.exe		- The executable built for Win95/NT with
  Microsoft Visual C++ 5.0.
sample.bat		- A batch file that invokes the cube solver with a
  sample cube configuration.
kcube.cpp               - Kociemba Cube Solver main program
cubepars.*		- CubeParser class, parses the command line
facecube.*		- FaceCube class, validates the a given cube
  configuration in its facelet representation.  Also converts
  the cube to a cubie permutation/orientation form.
solver.*		- Solver class, performs the two phase search
  a solution.
pruningt.*		- PruningTable class, generates and manages
  the pruning tables
kocimovt.*		- Contains a subclass of the MoveTable class
  for each of the five specific move tables.
movetabl.*		- MoveTable class, generates and manages the
  move mapping tables
kocicube.*		- KociembaCube class, a subclass of Cube with
  extensions to support Kociemba's algorithm.
cube.*			- Cube class, basic Rubik's cube model
combinat.*		- General purpose combinatorial routines.
vector.*		- General purpose Vector management routines.

Class Hierarchy
---------------

[CubeParser]
[FaceCube]
[Solver]
[PruningTable]
[MoveTable]
	|
	+- [TwistMoveTable]
	+- [FlipMoveTable]
	+- [ChoiceMoveTable]
	+- [CornerPermutationMoveTable]
	+- [NonMiddleSliceEdgePermutationMoveTable]
	+- [MiddleSliceEdgePermutationMoveTable]
[KociembaCube]
	|
	+- [Cube]

I would like to thank Herbert Kociemba for his patience
in providing me with enough details about his algorithm
to allow me to write this program.  I would also like to
credit Dik Winter as I implemented a version of his
choice numbering algorithm in the "KociembaCube" class,
as an alternate approach, for illustration.

Feedback of any kind, both positive and negative, regarding
  this program is welcome :)
 
Greg.Schmidt@ab.com
Work: (216) 646-3597
Home: (216) 481-6749

Greg Schmidt
498 Catlin Rd.
Richmond Hts., OH  44143
USA
