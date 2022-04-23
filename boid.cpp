#include "boid.h"

//Public Functions
//Constructors and Destructors
Boid::Boid(const int& WIDTH, const int& HEIGHT, const int& SIZE)
{
	std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> w(0,WIDTH);
    std::uniform_int_distribution<std::mt19937::result_type> h(0,HEIGHT);
    std::uniform_int_distribution<std::mt19937::result_type> d(-this->MAX_SS, this->MAX_SS);

    this->pos.first = w(rng);
    this->pos.second = h(rng);
    this->dir.first = d(rng);
    this->dir.second = d(rng);
    this->size = SIZE;
}

Boid::~Boid()
{
	
}

void Boid::printPos(){
    std::cout<<"("<<this->pos.first<<", "<<this->pos.second<<")\n";
}

void Boid::draw(sf::RenderWindow& window, const int& SIZE, bool& d){
    sf::CircleShape shape(SIZE);
    shape.setFillColor(this->bColour);
    shape.setPosition(sf::Vector2f(this->pos.first - (SIZE), this->pos.second - (SIZE)));
    window.draw(shape);
    if (d) { this->drawDir(window); }
}

void Boid::getLocalBoids(sf::RenderWindow& window, std::vector<Boid>& boids, bool& d){
    if (this->localBoids.size() > 0) { this->localBoids.clear(); }
    int i = -1;
    for (auto& b : boids){
        if ((this->pos != b.pos)){
            float d = dist(b);
            if (d < this->LOCAL_RAD){
                this->localBoids.push_back(b);
            }
        }
    }
    if(d) { this->connectLocal(window); }
}

//Steer to avoid crowding local boids
//Take average vector to all nearby boids and take opposite vector
void Boid::separation(sf::RenderWindow& window, bool& d){
    if (this->localBoids.size() > 0){
        this->s_vec.first = 0;
        this->s_vec.second = 0;
        
        for (auto& b : this->localBoids){
            this->s_vec.first += (b.get().pos.first - this->pos.first);
            this->s_vec.second += (b.get().pos.second - this->pos.second);
        }
        
        this->s_vec.first = (int)this->s_vec.first / (int)this->localBoids.size();
        this->s_vec.second = (int)this->s_vec.second / (int)this->localBoids.size();
        
        this->s_vec.first *= -1;
        this->s_vec.second *= -1;
        
        if(d) {
            sf::Vertex line[] =
            {
                sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), sf::Color::Magenta),
                sf::Vertex(sf::Vector2f(this->pos.first + this->s_vec.first, this->pos.second + this->s_vec.second), sf::Color::Magenta)
            };

            window.draw(line, 2, sf::Lines);
        }

        this->s_vec.first *= this->S_MULT;
        this->s_vec.second *= this->S_MULT;
    }
}

//Steer towards the average heading of local boids
//Take average vector of direction of all nearby boids
void Boid::alignment(sf::RenderWindow& window, bool& d){
    if (this->localBoids.size() > 0){
        this->a_vec.first = 0;
        this->a_vec.second = 0;
        
        for (auto& b : this->localBoids){
            this->a_vec.first += b.get().dir.first;
            this->a_vec.second += b.get().dir.second;
        }
        
        this->a_vec.first = (int)this->a_vec.first / (int)this->localBoids.size();
        this->a_vec.second = (int)this->a_vec.second / (int)this->localBoids.size();
        
        if(d) {
            sf::Vertex line[] =
            {
                sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), sf::Color::Green),
                sf::Vertex(sf::Vector2f(this->pos.first + this->a_vec.first, this->pos.second + this->a_vec.second), sf::Color::Green)
            };

            window.draw(line, 2, sf::Lines);
        }

        this->a_vec.first *= this->A_MULT;
        this->a_vec.second *= this->A_MULT;
    }
}

//Steer to move towards the average position of local boids
//Take vector to average position of all nearby boids
void Boid::cohesion(sf::RenderWindow& window, bool& d){
    if (this->localBoids.size() > 0){
        this->c_vec.first = 0;
        this->c_vec.second = 0;   
        
        for (auto& b : this->localBoids){
            this->c_vec.first += b.get().pos.first;
            this->c_vec.second += b.get().pos.second;
        }

        this->c_vec.first = this->c_vec.first / this->localBoids.size();
        this->c_vec.second = this->c_vec.second / this->localBoids.size();

        this->c_vec.first -= this->pos.first;
        this->c_vec.second -= this->pos.second;

        if(d) {
            sf::Vertex line[] =
            {
                sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), sf::Color::Cyan),
                sf::Vertex(sf::Vector2f(this->pos.first + this->c_vec.first, this->pos.second + this->c_vec.second), sf::Color::Cyan)
            };

            window.draw(line, 2, sf::Lines);
        }

        this->c_vec.first *= this->C_MULT;
        this->c_vec.second *= this->C_MULT;

    }
}

void limit(std::pair<int, int>& p, const int& max){
    if (((p.first * p.first) + (p.second * p.second)) > max){
        int x = round((float)p.first / (float)((p.first * p.first) + (p.second * p.second)));
        int y = round((float)p.second / (float)((p.first * p.first) + (p.second * p.second)));
        p = std::make_pair(x * max, y * max);
    }
}

void Boid::getNewDirection(){
    std::pair<int, int> avg = {0, 0};
    avg.first += (a_vec.first + c_vec.first + s_vec.first) / 3.0;
    avg.second += (a_vec.second + c_vec.second + s_vec.second) / 3.0;
    avg.first *= this->AVG_MULT;
    avg.second *= this->AVG_MULT;
    if (avg.first != 0){
        this->dir.first = (this->dir.first + avg.first);
    }
    if (avg.second != 0){
        this->dir.second = (this->dir.second + avg.second);
    }
    limit(this->dir, this->MAX_VEL);

    if(this->ADD_RANDOM){
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> r(-5,5);
        this->dir.first += r(rng);
        this->dir.second += r(rng);
    }
}

void Boid::move(const int& WIDTH, const int& HEIGHT){
    this->pos.first += std::ceil((float)this->dir.first * 0.1);
    this->pos.second += std::ceil((float)this->dir.second * 0.1);

    if ((this->pos.first > WIDTH) || (this->pos.first < 0)){ this->dir.first *= -1; }
    if ((this->pos.second > HEIGHT) || (this->pos.second < 0)){ this->dir.second *= -1; }
    if (this->pos.first > WIDTH + this->size){ this->pos.first = WIDTH; }
    if (this->pos.first < -this->size){ this->pos.first = 0; }
    if (this->pos.second > HEIGHT + this->size){ this->pos.second = 0; }
    if (this->pos.second < -this->size){ this->pos.second = HEIGHT; }

}

//Private Functions

void Boid::drawDir(sf::RenderWindow& window){
    sf::Vertex line[] =
    {
        sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), this->dColour),
        sf::Vertex(sf::Vector2f(this->pos.first + this->dir.first, this->pos.second + this->dir.second), this->dColour)
    };

    window.draw(line, 2, sf::Lines);
}

float Boid::dist(Boid& b){
    float x = (float)this->pos.first - b.pos.first;
    float y = (float) this->pos.second - b.pos.second;
    float dist = (x * x) + (y * y);
    return dist;
}

void Boid::connectLocal(sf::RenderWindow& window){
    for (auto& b : this->localBoids){
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), this->cColour),
            sf::Vertex(sf::Vector2f(b.get().pos.first, b.get().pos.second), this->cColour)
        };

        window.draw(line, 2, sf::Lines);
    }
}