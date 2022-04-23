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
        void draw(sf::RenderWindow& window, const int& SIZE);
        void getLocalBoids(std::vector<Boid>& boids);
        void separation(sf::RenderWindow& window);
        void alignment(sf::RenderWindow& window);
        void cohesion(sf::RenderWindow& window);
        void getNewDirection();
        void move(const int& WIDTH, const int& HEIGHT);
        

    private:
        const int MAX_SS = 50;
        //Square to remove squaring in limit function
        //e.g. MAX_VEl = 25 so max vel is actually 5
        const int MAX_VEL = 1600;
        //Distance away squared
        //Removes sqrt() in dist function
        const float LOCAL_RAD = 2500.0;
        const float C_MULT = 0.5;
        const float S_MULT = 0.1;
        const float A_MULT = 0.5;
        const float AVG_MULT = 0.3;
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