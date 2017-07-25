
//Efe Ulas Akay Seyitoglu & Huseyin Beyan email: euase@kth.se & huseyinb@kth.se
#include <iostream>

#include "glm/glm.hpp"

#include <SDL.h>

#include "SDLauxiliary.h"

#include "TestModel.h"



using namespace std;

using glm::vec3;

using glm::mat3;



// ----------------------------------------------------------------------------

// GLOBAL VARIABLES



const int SCREEN_WIDTH = 100;

const int SCREEN_HEIGHT = 100;

SDL_Surface* screen;

int t;

float focalLength = SCREEN_HEIGHT;

vec3 cameraPos(0,0,-3);

mat3 R;

vec3 lightPos( 0, -0.5, -0.7 );

vec3 lightColor = 14.f * vec3( 1, 1, 1 );

float yaw = 0;



// ----------------------------------------------------------------------------

// FUNCTIONS



void Update();

void Draw();





vector<Triangle> triangles;





struct Intersection

{
    
    vec3 position;
    
    float distance;
    
    int triangleIndex;
    
};

float returnMax(float x,float y) {
    
    if(x > y)
        
        return x;
    
    return y;
    
    
    
}

vec3 DirectLight( const Intersection& i );

bool ClosestIntersection(
                         
                         vec3 start,
                         
                         vec3 dir,
                         
                         const vector<Triangle>& triangles,
                         
                         Intersection& closestIntersection
                         
                         );









int main( int argc, char* argv[] )

{
    
    screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
    
    t = SDL_GetTicks();	// Set start value for timer.
    
    
    
    
    
    LoadTestModel(triangles);
    
    while( NoQuitMessageSDL() )
        
    {
        
        Update();
        
        Draw();
        
    }
    
    
    
    SDL_SaveBMP( screen, "screenshot.bmp" );
    
    
    
    
    
    
    
    return 0;
    
}



bool ClosestIntersection(vec3 start,vec3 dir,vector<Triangle>& triangles,Intersection& closestIntersection)

{
    
    bool var = false;
    
    closestIntersection.distance = std::numeric_limits<float>::max();
    
    for(int i = 0; i < triangles.size();i++) {
        
        
        
        vec3 v0 = R*triangles[i].v0;
        
        vec3 v1 = R*triangles[i].v1;
        
        vec3 v2 = R*triangles[i].v2;
        
        vec3 e1 = v1 - v0;
        
        vec3 e2 = v2 - v0;
        
        vec3 b = start - v0;
        
        mat3 A( -dir, e1, e2 );
        
        vec3 x = glm::inverse( A ) * b;
        
        vec3 r =v0 + x.y*e1 + e2*x.z;
        
        if(x.x > 0 && x.y >= 0 && x.z >= 0 && x.y + x.z < 1 ) {
            
            var = true;
            
            if(x.x < closestIntersection.distance) {
                
                closestIntersection.position = r;
                
                closestIntersection.distance = x.x;
                
                closestIntersection.triangleIndex = i;
                
            }
            
            
            
        }
        
        
        
        
        
    }
    
    return var;
    
}



void Update()

{
    
    
    
    // Compute frame time:
    
    int t2 = SDL_GetTicks();
    
    float dt = float(t2-t);
    
    t = t2;
    
    cout << "Render time: " << dt << " ms." << endl;
    
    Uint8* keystate = SDL_GetKeyState( 0 );
    
    if( keystate[SDLK_UP] )
        
    {
        
        // Move camera forward
        
        cameraPos.z +=0.1;
        
    }
    
    if( keystate[SDLK_DOWN] )
        
    {
        
        // Move camera backward
        
        cameraPos.z -=0.1;
        
    }
    
    if( keystate[SDLK_LEFT] )
        
    {
        
        // Move camera to the left
        
        yaw = yaw - 0.1;
        
        
        
    }
    
    if( keystate[SDLK_RIGHT] )
        
    {
        
        // Move camera to the right
        
        yaw = yaw + 0.1;
        
        
        
    }
    
    
    
    if( keystate[SDLK_w] )
        
        lightPos.z += 0.1;
    
    if( keystate[SDLK_s] )
        
        lightPos.z -= 0.1;
    
    if( keystate[SDLK_d] )
        
        lightPos.x -= 0.1;
    
    if( keystate[SDLK_a] )
        
        lightPos.x += 0.1;
    
    if( keystate[SDLK_q] )
        
        lightPos.y -= 0.1;
    
    if( keystate[SDLK_e] )
        
        lightPos.y += 0.1;
    
    
    
    R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0,-sin(yaw), 0, cos(yaw));
    
}



void Draw()

{
    
    if( SDL_MUSTLOCK(screen) )
        
        SDL_LockSurface(screen);
    
    
    
    vec3 indirectLight = 0.5f*vec3( 1, 1, 1 );
    
    
    
    
    
    for( int y=0; y<SCREEN_HEIGHT; ++y )
        
    {
        
        for( int x=0; x<SCREEN_WIDTH; ++x )
            
        {
            
            vec3 d(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength);
            
            Intersection localInter;
            
            if(ClosestIntersection(cameraPos, d, triangles, localInter)) {
                
                vec3 d2 = DirectLight(localInter);
                
                PutPixelSDL( screen, x, y, (indirectLight+d2)*triangles[localInter.triangleIndex].color );
                
            }
            else {
                
                
                
                vec3 color(0,0,0);
                
                PutPixelSDL( screen, x, y, color);
                
            }
            
        }
        
    }
    
    
    
    if( SDL_MUSTLOCK(screen) )
        
        SDL_UnlockSurface(screen);
    
    
    
    SDL_UpdateRect( screen, 0, 0, 0, 0 );
    
}







vec3 DirectLight( const Intersection& i ){
    
    
    
    vector<Triangle> triangles2;
    
    triangles2= triangles;
    
    triangles2.erase(triangles2.begin() + i.triangleIndex);
    
    
    
    vec3 normal = triangles[i.triangleIndex].normal;
    
    float radius = glm::distance(lightPos,i.position);
    vec3 r = glm::normalize(lightPos-i.position);
    float surfaceArea = 4*3.1415*radius*radius;
    vec3 d = lightColor*returnMax(glm::dot(r,normal),0)/surfaceArea;
    Intersection localInter;
    // vec3 d2(i. - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, focalLength);
    bool check = ClosestIntersection(i.position, r, triangles2, localInter);
    
    
    
    if(check) {
        
        if(glm::distance(localInter.distance , i.distance ) < radius) {
            
            vec3 a(0,0,0);
            
            return a;
        }
    }
    
    
    
    return d;
    
}
