/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

namespace Engine.Box2D;

struct Body
{

    public Body()
    {
        position = new Vec2(0.0f, 0.0f);
        rotation = 0.0f;
        velocity = new Vec2(0.0f, 0.0f);
        angularVelocity = 0.0f;
        force = new Vec2(0.0f, 0.0f);
        torque = 0.0f;
        friction = 0.2f;

        width = new Vec2(1.0f, 1.0f);
        mass = float.MaxValue;
        invMass = 0.0f;
        I = float.MaxValue;
        invI = 0.0f;
    }

    void Set(in Vec2 w, float m)
    {
        position.Set(0.0f, 0.0f);
        rotation = 0.0f;
        velocity.Set(0.0f, 0.0f);
        angularVelocity = 0.0f;
        force.Set(0.0f, 0.0f);
        torque = 0.0f;
        friction = 0.2f;

        width = w;
        mass = m;

        if (mass < float.MaxValue)
        {
            invMass = 1.0f / mass;
            I = mass * (width.x * width.x + width.y * width.y) / 12.0f;
            invI = 1.0f / I;
        }
        else
        {
            invMass = 0.0f;
            I = float.MaxValue;
            invI = 0.0f;
        }
    }

    public void AddForce(in Vec2 f)
    {
        force += f;
    }

    Vec2 position;
    float rotation;

    Vec2 velocity;
    float angularVelocity;

    Vec2 force;
    float torque;

    Vec2 width;

    float friction;
    float mass, invMass;
    float I, invI;
};