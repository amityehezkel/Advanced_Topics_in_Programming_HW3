#include <Simulator/CommandLineParser.h>
#include <Simulator/SimulatorApplication.h>

#include <exception>
#include <iostream>

int main(int argc, char* argv[])
{
    try {
        const auto options = simulator::CommandLineParser::parse(argc, argv);
        return simulator::SimulatorApplication{std::cout, std::cerr}.run(options);
    } catch (const simulator::CommandLineError& error) {
        std::cerr << error.what() << "\n\n"
                  << simulator::CommandLineParser::usage() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Simulator failed: " << error.what() << "\n\n"
                  << simulator::CommandLineParser::usage() << '\n';
    } catch (...) {
        std::cerr << "Simulator failed with an unknown error.\n\n"
                  << simulator::CommandLineParser::usage() << '\n';
    }
    return 1;
}
