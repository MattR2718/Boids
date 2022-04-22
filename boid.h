#ifndef BOID
#define BOID

#include <random>
#include <iostream>
#include <SFML/Graphics.hpp>

class Boid {
    public:
        //Constructors and destructors
        Boid(const int& WIDTH, const int& HEIGHT);
        virtual ~Boid();

        void printPos();
        void draw(sf::RenderWindow& window, const int& SIZE);
        void getLocalBoids(std::vector<Boid>& boids);
        void separation();
        void alignment();
        void cohesion();
        void getNewDirection();
        void move();
        

    private:
        const int INIT_VEL = 50;
        const int MAX_SS = 50;
        const float LOCAL_RAD = 50.0;
        const float C_MULT = 0.5;
        const float S_MULT = 0.5;
        const float A_MULT = 0.5;
        sf::Color bColour = sf::Color::Yellow;
        sf::Color cColour = sf::Color::Blue;
        sf::Color dColour = sf::Color::Red;
        std::pair<int, int> pos;
        std::pair<int, int> dir;
        std::pair<int, int> s_vec = {0, 0};
        std::pair<int, int> c_vec = {0, 0};
        std::pair<int, int> a_vec = {0, 0};
        std::vector<Boid> localBoids;

        void drawDir(sf::RenderWindow& window);
        float dist(Boid& b);
        void connectLocal(sf::RenderWindow& window);

};

#endif