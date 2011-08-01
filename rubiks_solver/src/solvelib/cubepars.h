// cubepars.h - Cube Parser class definition
//
// Parse the cube state passed in on the command line
// checking for gross syntax errors.  For example, all
// 9 facelet markings for each of the six sides must
// be specified, and markings must be a printable ASCII
// character.  If the parse was successful a "FaceletCube"
// is initialized.  The FaceletCube represents the cube
// by the markings of the 54 individual facelets.  The
// FaceletCube can then be asked to validate the cube
// to determine if it is in a legal, and thus solvable,
// configuration.
//
#ifndef	_cubepars_h
#define	_cubepars_h

#include <string>
#include <sstream>
class FaceletCube;

class CubeParser
{
public:
	// Parser return codes
	enum {
		VALID,
		INVALID_FACE,
		INVALID_MARKER,
		INCOMPLETE_INPUT,
		SYNTAX_ERROR,
		NumberOfErrors };

	CubeParser(void);
	~CubeParser();

	// Parse the input and initialize a FaceletCube
	int ParseInput(int argc, const char* argv[], FaceletCube& faceletCube);

	int ParseInputROS(std::string &input,  FaceletCube& faceletCube);

	// Return the text associated with an error return code
	char* ErrorText(unsigned int error);

private:
	// Return the text associated with an error return code
	int ParseFace(FaceletCube& faceletCube, const char* faceString, int& face);

	static char* errorText[NumberOfErrors];
};

#endif // _cubepars_h
