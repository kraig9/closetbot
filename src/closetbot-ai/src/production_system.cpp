#include <iostream>

class ProductionSystem
{
public:
    void run(){
        std::cout << "Running Production System" << std::endl;
    }
};

int main()
{
    std::cout << "Hello World!\n";
    ProductionSystem p;
    p.run();
}