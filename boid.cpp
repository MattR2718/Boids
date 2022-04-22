#include "boid.h"

//Public Functions
//Constructors and Destructors
Boid::Boid(const int& WIDTH, const int& HEIGHT)
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

}

Boid::~Boid()
{
	
}

void Boid::printPos(){
    std::cout<<"("<<this->pos.first<<", "<<this->pos.second<<")\n";
}

void Boid::draw(sf::RenderWindow& window, const int& SIZE){
    sf::CircleShape shape(SIZE);
    shape.setFillColor(this->bColour);
    shape.setPosition(sf::Vector2f(this->pos.first - (SIZE), this->pos.second - (SIZE)));
    window.draw(shape);
    this->drawDir(window);
    this->connectLocal(window);
}

void Boid::getLocalBoids(std::vector<Boid>& boids){
    int i = -1;
    //std::cout.precision(3);
    for (auto& b : boids){
        if ((this->pos != b.pos)){
            float d = dist(b);
            //std::cout<<typeid(d).name()<<'\n';
            if (d < this->LOCAL_RAD){
                //float test = 3.6;
                //std::cout.precision(3);
                //std::cout<<++i<<": "<<d<<"      test: "<<test<<'\n';
                //this->connect(window, b);
                //this->colour = sf::Color::Green;
                this->localBoids.push_back(b);
            }
        }
    }
}

//Steer to avoid crowding local boids
//Take average vector to all nearby boids and take opposite vector
void Boid::separation(){
    if (this->localBoids.size() > 0){
        this->s_vec.first = 0;
        this->s_vec.second = 0;
        
        for (auto& b : this->localBoids){
            this->s_vec.first += (b.pos.first - this->pos.first);
            this->s_vec.second += (b.pos.second - this->pos.second);
        }
        
        
        this->s_vec.first = (int)this->s_vec.first / (int)this->localBoids.size();
        this->s_vec.second = (int)this->s_vec.second / (int)this->localBoids.size();
        
        this->s_vec.first *= -1;
        this->s_vec.second *= -1;

        //std::cout<<"x: "<<this->s_vec.first<<"  y: "<<this->s_vec.second<<"  size: "<<this->localBoids.size()<<'\n';

        this->s_vec.first *= this->S_MULT;
        this->s_vec.second *= this->S_MULT;
        
    }
}

//Steer towards the average heading of local boids
//Take average vector of direction of all nearby boids
void Boid::alignment(){
    if (this->localBoids.size() > 0){
        this->a_vec.first = 0;
        this->a_vec.second = 0;
        
        for (auto& b : this->localBoids){
            this->a_vec.first += b.dir.first;
            this->a_vec.second += b.dir.first;
        }
        
        this->a_vec.first = (int)this->a_vec.first / (int)this->localBoids.size();
        this->a_vec.second = (int)this->a_vec.second / (int)this->localBoids.size();
        
        this->a_vec.first *= this->A_MULT;
        this->a_vec.second *= this->A_MULT;
    }
}

//Steer to move towards the average position of local boids
//Take vector to average position of all nearby boids
void Boid::cohesion(){
    if (this->localBoids.size() > 0){
        this->c_vec.first = 0;
        this->c_vec.second = 0;
        
        for (auto& b : this->localBoids){
            this->c_vec.first += b.pos.first;
            this->c_vec.second += b.pos.first;
        }

        this->c_vec.first = (int)this->c_vec.first / (int)this->localBoids.size();
        this->c_vec.second = (int)this->c_vec.second / (int)this->localBoids.size();

        this->c_vec.first *= this->C_MULT;
        this->c_vec.second *= this->C_MULT;

    }
}

void Boid::getNewDirection(){
    std::pair<int, int> avg = {0, 0};
    avg.first += (a_vec.first + c_vec.first + s_vec.first);
    avg.second += (a_vec.second + c_vec.second + s_vec.second);
    if (avg.first != 0){
        this->dir.first = (this->dir.first + avg.first) / 2;
    }
    if (avg.second != 0){
        this->dir.second = (this->dir.second + avg.second) / 2;
    }
    std::cout<<this->dir.first<<", "<<this->dir.second<<'\n';
}

void Boid::move(){
    this->pos.first += (this->dir.first * 0.1);
    this->pos.second += (this->dir.second * 0.1);
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
    float dist = sqrt((x * x) + (y * y));
    //dist = sqrt(3.0);
    //int f = (dist < 3.0) ? 1 : 0;
    //std::cout<<"X: "<<x<<"  Y: "<<y<<"  DIST: "<<dist<<"  T/F: "<<f<<"\n";
    return dist;
}

void Boid::connectLocal(sf::RenderWindow& window){
    for (auto& b : this->localBoids){
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(this->pos.first, this->pos.second), this->cColour),
            sf::Vertex(sf::Vector2f(b.pos.first, b.pos.second), this->cColour)
        };

        window.draw(line, 2, sf::Lines);
    }
}