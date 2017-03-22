#include <iostream>
#include <string>
//#include <vector>

/*class Parser{
	public:
		void parse(std::string, float&, float&);
		//std::vector<std::string> parse_music(std::string);
};*/

void parse(std::string in, float& r, float& v);
/*std::vector<std::string> Parser::parse_music(std::string in){
	return 0;
}*/

int main(){
	std::string input;
	float r,v;

	while(true){
		std::cout << "Input: " << std::endl;
		std::cin >> input;

		if(input == "q"){
			std::cout << "Exiting" << std::endl;
			break;
		}

		parse(input, r, v);

		if(v == -2.0){
			std::cout << "R: " << r  << std::endl;
		}
		else if(r == -2.0){
			std::cout << "V: " << v << std::endl;
		}
		else if(r == -1.0 && v == -1.0){
			std::cout << "Invalid Command!" << std::endl;
		}
		else{
			std::cout << "R: " << r << " V: " << v << std::endl;
		}


	}
	return 0;
}

void parse(std::string in, float& r, float& v){
	if((in.find("R") != std::string::npos) && (in.find("V") != std::string::npos)){	// R and V command
		r = std::stof(in.substr(in.find("R")+1, in.find("V")-1));
		v = std::stof(in.substr(in.find("V")+1));
		if(v < 0){
			v *= -1;
		}
	}
	else if(in.find("V") != std::string::npos){ // Only V command
		v = std::stof(in.substr(in.find("V")+1));
		r = -2.0; // Used for when R does not change
	}
	else if(in.find("R") != std::string::npos){ // Only R command
		r = std::stof(in.substr(in.find("R")+1));
		v = -2.0; // Used for when V does not change
	}
	else{
		r,v = -1.0;	// Used for invalid input commands, deal with at higher level
	}
}