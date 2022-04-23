#ifndef BOID
#define BOID

#include <random>
#include <iostream>
#include <functional>
#include <SFML/Graphics.hpp>

class Boid {
    public:
        //Constructors and destructors
        Boid(const int& WIDTH, const int& HEIGHT);
        virtual ~Boid();

        void printPos();
        void draw(sf::RenderWindow& window, const int& SIZE);
        void getLocalBoids(std::vector<Boid>& boids);
        void separation(sf::RenderWindow& window);
        void alignment();
        void cohesion();
        void getNewDirection();
        void move();
        

    private:
        const int MAX_SS = 50;
        //Distance away squared
        //Removes sqrt() in dist function
        const float LOCAL_RAD = 2500.0;
        const float C_MULT = 0.5;
        const float S_MULT = 0.5;
        const float A_MULT = 0.2;
        const float AVG_MULT = 0.2;
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