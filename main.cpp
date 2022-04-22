#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>

#include "boid.h"

float fps;
sf::Clock fps_clock = sf::Clock();
sf::Time previous_time = fps_clock.getElapsedTime();
sf::Time current_time;

int main()
{
    const int WIDTH = 800;
    const int HEIGHT = 800;
    const int NUM = 50;
    const int SIZE = 10;

    int fps;
    sf::Text text;
    sf::Font font;
    if(!font.loadFromFile("arial.ttf")){ std::cout<<"ERROR LOADING FONT\n"; }
    text.setFont(font);
    text.setCharacterSize(30);
    text.setFillColor(sf::Color::White);
    text.setPosition(sf::Vector2f(0, 0));
    
    
    std::vector<Boid> boids;
    for (int i = 0; i < NUM; i++){
        Boid temp(WIDTH, HEIGHT);
        //temp.printPos();
        boids.push_back(temp);
    }

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "BOIDS");
    window.setFramerateLimit(30);
    while (window.isOpen())
    {
        current_time = fps_clock.getElapsedTime();
		fps = int(1.f / (current_time.asSeconds() - previous_time.asSeconds()));
        text.setString(std::to_string(fps));
		//std::cout << fps << '\n';
		previous_time = current_time;

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed){
                window.close();
            }
        }
        window.clear();
        
        for (auto& b : boids){
            b.getLocalBoids(boids);
            //b.separation();
            //b.alignment();
            //b.cohesion();
            //b.getNewDirection();
            //b.move();
            b.draw(window, SIZE);
        }

        window.draw(text);
        window.display();
    }
}