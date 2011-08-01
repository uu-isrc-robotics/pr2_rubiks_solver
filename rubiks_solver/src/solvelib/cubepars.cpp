// cubepars.cpp - Cube Parser class implementation

#include "cubepars.h"	// CubeParser
#include "facecube.h"	// FaceletCube
#include <string.h>

CubeParser::CubeParser(void)
{
	// Nothing to do here
}

CubeParser::~CubeParser()
{
	// Nothing to do here
}

//// Parse the input and initialize a FaceletCube
//int CubeParser::ParseInput(std::string &layout, FaceletCube& faceletCube)
//{
//	int face;
//	int faces[6];
//	int i;
//	int status;
//
//
//
//	// All six face specifiers must be present
//	if (argc != 6+1) return SYNTAX_ERROR;
//
//	// Reset the face count for all faces
//	for (i = 0; i < 6; i++) faces[i] = 0;
//
//	// Loop through each face specifier
//	for (i = 0; i < 6; i++)
//	{
//		// Parse each face
//		if ((status = ParseFace(faceletCube, argv[i+1], face)) != VALID)
//			return status;
//
//		// Initialize this face in the FaceletCube
//		faceletCube.SetFaceMarkings(face, argv[i+1]+2);
//
//		// Count this face
//		faces[face]++;
//	}
//
//	// Each face specifier must be found exactly once
//	for (i = 0; i < 6; i++)
//	{
//		if (faces[i] != 1)
//			return INCOMPLETE_INPUT;
//	}
//
//	return VALID;
//}


int CubeParser::ParseInputROS(std::string &input, FaceletCube& faceletCube) {
	std::stringstream str_strm;
	str_strm << input;
	std::string facestr;
	int face;
	int status;
	// Loop through each face specifier
	for (int i = 0; i < 6; i++)
	{
		str_strm >> facestr;
		const char *part = facestr.c_str();
		// Parse each face
		if ((status = ParseFace(faceletCube, part, face)) != VALID)
			return status;

		// Initialize this face in the FaceletCube
		faceletCube.SetFaceMarkings(face, part+2);

	}

	return VALID;

}
// Parse the input and initialize a FaceletCube
int CubeParser::ParseInput(int argc, const char* argv[], FaceletCube& faceletCube)
{
	int face;
	int faces[6];
	int i;
	int status;

	// All six face specifiers must be present
	if (argc != 6+1) return SYNTAX_ERROR;

	// Reset the face count for all faces
	for (i = 0; i < 6; i++) faces[i] = 0;

	// Loop through each face specifier
	for (i = 0; i < 6; i++)
	{
		// Parse each face
		if ((status = ParseFace(faceletCube, argv[i+1], face)) != VALID)
			return status;

		// Initialize this face in the FaceletCube
		faceletCube.SetFaceMarkings(face, argv[i+1]+2);
		
		// Count this face
		faces[face]++;
	}
		
	// Each face specifier must be found exactly once
	for (i = 0; i < 6; i++)
	{
		if (faces[i] != 1)
			return INCOMPLETE_INPUT;
	}
	
	return VALID;
}

// Return the text associated with an error return code
int CubeParser::ParseFace(FaceletCube& faceletCube, const char* faceString, int& face)
{
	int facelet;

	// Check specifier length f:mmmmmmmmm
	if (strlen(faceString) != 1+1+9)
		return SYNTAX_ERROR;

	// Validate face name (f)
	if ((face = faceletCube.FaceNameToOffset(faceString[0])) < 0)
		return INVALID_FACE;
	
	// Parse the colon
	if (faceString[1] != ':')
		return SYNTAX_ERROR;
	
	// Check each facelet
	for (facelet = 0; facelet < 9; facelet++)
	{
		// Only printable characters are allowed
		if (faceString[2+facelet] <= ' ' ||
			faceString[2+facelet] > '~')
			return SYNTAX_ERROR;
	}

	return VALID;
}

// Return the text associated with an error return code
char* CubeParser::ErrorText(unsigned int error)
{
	if (error >= NumberOfErrors) error = 0;
	return errorText[error];
}

char* CubeParser::errorText[NumberOfErrors] = {
	(char*)"",
	(char*)"Invalid face specifier",
	(char*)"Invalid marker",
	(char*)"Incomplete input",
	(char*)"Syntax error" };

