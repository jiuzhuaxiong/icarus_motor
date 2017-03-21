#include <regex>
#include <iostream>
#include <string>

float parseR_V(std::string in);
float parseR(std::string in);
float parseV(std::string in);

int main()
{
	std::string input;
	std::regex r_r ("(R-?\\d{1,3}(\\.\\d{1,2})?)");
	std::regex v_r ("(V\\d{1,3}(\\.\\d{1,2})?)");
	std::regex cmd_r ("(R-?\\d{1,3}(\\.\\d{1,2})?)?(V\\d{1,3}(\\.\\d{1,2})?)?");
	std::regex music_r ("T([A-G][#\\^]?[1-8]){1,16}");


	//(R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,2})?)?

	//T([A-G][#\^]?[1-8]){1,16}

	while(true){

		std::cout << "Input Command: " << std::endl;
		std::cin >> input;

		if(input=="q")
			break;

		std::smatch rm;
		std::smatch vm;
		std::smatch cm;
		std::smatch mm;

		std::cout << "Input was: " << input << std::endl;

		std::regex_match (input, rm, r_r);

		std::cout << "R: " << rm[0] << std::endl;

		std::regex_match (input, vm, v_r);

		std::cout << "V: " << vm[0] << std::endl;

		std::regex_match (input, cm, cmd_r);

		std::cout << "Cmd: " << cm[0] << std::endl;

		std::regex_match (input, mm, music_r);

		std::cout << "Music: " << mm[0] << std::endl;

		float r = 0.0;
		float v = 0.0;

		
		if(std::regex_match (input, r_r)){
			r = parseR(input);
			std::cout << "R: " << r << std::endl;
		}
		else if(std::regex_match (input, v_r)){
			v = parseV(input);
			std::cout << "V: " << v << std::endl;
		}
		else if (std::regex_match (input, cmd_r)){
			r = parseR_V(input);
			v = parseV(input);
			std::cout << "R: " << r << " V: " << v << std::endl;
		}
		std::cout << "Done" << std::endl;
	}
	return 0;
}

float parseR_V(std::string in)
{
	std::string tokenR = in.substr(in.find("R")+1, in.find("V")-1);
	return std::stof(tokenR);
}

float parseR(std::string in)
{
	std::string tokenR = in.substr(in.find("R")+1, std::string::npos);
	return std::stof(tokenR);
}

float parseV(std::string in)
{
	std::string tokenV = in.substr(in.find("V")+1, std::string::npos);
	return std::stof(tokenV);
}