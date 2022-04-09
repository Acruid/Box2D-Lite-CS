/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

using System.Runtime.InteropServices;

namespace Engine.Box2D;

struct World
{
    World(Vec2 gravity, int iterations)
    {
        this.gravity = gravity;
        this.iterations = iterations;
        bodies = new List<Body>();
        joints = new List<Joint>();
        arbiters = new Dictionary<ArbiterKey, Arbiter>();
    }

    void Add(ref Body body)
    {
        bodies.Add(body);
    }

    void Add(ref Joint joint)
    {
        joints.Add(joint);
    }

    void Clear()
    {
        bodies.Clear();
        joints.Clear();
        arbiters.Clear();
    }

    void Step(float dt)
    {
        float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

        // Determine overlapping bodies and update contact points.
        BroadPhase();
        
        // don't modify bodies while in use
        var spanBodies = CollectionsMarshal.AsSpan(bodies);

        // Integrate forces.
        for (int i = 0; i < (int)spanBodies.Length; ++i)
        {
            ref Body b = ref spanBodies[i];

            if (b.invMass == 0.0f)
                continue;

            b.velocity += dt * (gravity + b.invMass * b.force);
            b.angularVelocity += dt * b.invI * b.torque;
        }

        // Perform pre-steps.
        foreach (var kvArbiter in arbiters)
        {
            kvArbiter.Value.PreStep(inv_dt);
        }

        for (int i = 0; i < joints.Count; ++i)
        {
            joints[i].PreStep(inv_dt);	
        }

        // Perform iterations
        for (int i = 0; i < iterations; ++i)
        {
            foreach (var kvArbiter in arbiters)
            {
                kvArbiter.Value.ApplyImpulse();
            }

            for (int j = 0; j < (int)joints.Count; ++j)
            {
                joints[j].ApplyImpulse();
            }
        }

        // Integrate Velocities
        for (int i = 0; i < spanBodies.Length; ++i)
        {
            ref Body b = ref spanBodies[i];

            b.position += dt * b.velocity;
            b.rotation += dt * b.angularVelocity;

            b.force.Set(0.0f, 0.0f);
            b.torque = 0.0f;
        }
    }

    void BroadPhase()
    {
        // don't modify bodies while in use
        var spanBodies = CollectionsMarshal.AsSpan(bodies);

        // O(n^2) broad-phase
        for (int i = 0; i < spanBodies.Length; ++i)
        {
            ref Body bi = ref spanBodies[i];

            for (int j = i + 1; j < spanBodies.Length; ++j)
            {
                ref Body bj = ref spanBodies[j];

                if (bi.invMass == 0.0f && bj.invMass == 0.0f)
                    continue;

                Arbiter newArb = new(bi, bj);
                ArbiterKey key = new(bi, bj);

                if (newArb.numContacts > 0)
                {
                    if(!arbiters.TryGetValue(key, out var arb))
                    {
                        arbiters.Add(key, newArb);
                    }
                    else
                    {
                        arb.Update(newArb.contacts, newArb.numContacts);
                    }
                }
                else
                {
                    arbiters.Remove(key);
                }
            }
        }
    }

    readonly List<Body> bodies;
    readonly List<Joint> joints;
    readonly Dictionary<ArbiterKey, Arbiter> arbiters;
    Vec2 gravity;
    int iterations;

    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
};