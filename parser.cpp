#include <iostream>
#include <string>

class Parser{
	public:
		float parse(string);
		vector<string> parse_music(string);
};

float Parser::parse(string in){
	if in.find("R") && in.find("V")
		return std::stof(in.substr(in.find("R")+1, in.find("V")-1));

	else if in.find("V")
		return std:stof(in.substr(in.find("V")+1));

	else if in.find("R")
		return std::stof(in.substr(in.find("R")+1));

	else return -1;
}

vector<string> Parser::parse_music(string in){
	return null;
}