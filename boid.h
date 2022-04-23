#ifndef BOID
#define BOID

#include <random>
#include <iostream>
#include <functional>
#include <SFML/Graphics.hpp>

class Boid {
    public:
        //Constructors and destructors
        Boid(const int& WIDTH, const int& HEIGHT, const int& SIZE);
        virtual ~Boid();

        void printPos();
        void draw(sf::RenderWindow& window, const int& SIZE, bool& d);
        void getLocalBoids(sf::RenderWindow& window, std::vector<Boid>& boids, bool& d);
        void separation(sf::RenderWindow& window, bool& d);
        void alignment(sf::RenderWindow& window, bool& d);
        void cohesion(sf::RenderWindow& window, bool& d);
        void getNewDirection();
        void move(const int& WIDTH, const int& HEIGHT);
        

    private:
        const int MAX_SS = 50;
        //Square to remove squaring in limit function
        //e.g. MAX_VEl = 25 so max vel is actually 5
        const int MAX_VEL = 2500;
        //Distance away squared
        //Removes sqrt() in dist function
        const float LOCAL_RAD = 10000.0;
        const float C_MULT = 0.5;
        const float S_MULT = 0.1;
        const float A_MULT = 0.5;
        const float AVG_MULT = 0.3;
        const bool ADD_RANDOM = false;
        int size;
        sf::Color bColour = sf::Color::Yellow;
        sf::Color cColour = sf::Color::Blue;
        sf::Color dColour = sf::Color::Red;
        std::pair<int, int> pos;
        std::pair<int, int> dir;
        std::pair<int, int> s_vec = {0, 0};
        std::pair<int, int> c_vec = {0, 0};
        std::pair<int, int> a_vec = {0, 0};
        std::vector<std::reference_wrapper<Boid>> localBoids;

        void drawDir(sf::RenderWindow& window);
        float dist(Boid& b);
        void connectLocal(sf::RenderWindow& window);
};

#endif