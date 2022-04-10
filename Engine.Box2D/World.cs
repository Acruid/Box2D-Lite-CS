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
    public World(Vec2 gravity, int iterations)
    {
        this.gravity = gravity;
        this.iterations = iterations;
        arbiters = new Dictionary<ArbiterKey, Arbiter>();
    }

    public void Clear()
    {
        arbiters.Clear();
    }

    public void Step(float dt)
    {
        float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;
        
        // don't modify bodies while in use
        var spanBodies = new Span<Body>(Program.bodies, 0, Program.numBodies);
        var joints = new Span<Joint>(Program.joints, 0, Program.numJoints);

        // Determine overlapping bodies and update contact points.
        BroadPhase(spanBodies);

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
            ref Arbiter arb = ref CollectionsMarshal.GetValueRefOrNullRef(arbiters, kvArbiter.Key);
            arb.PreStep(spanBodies, inv_dt);
        }

        for (int i = 0; i < joints.Length; ++i)
        {
            joints[i].PreStep(spanBodies, inv_dt);	
        }

        // Perform iterations
        for (int i = 0; i < iterations; ++i)
        {
            foreach (var kvArbiter in arbiters)
            {
                ref Arbiter arb = ref CollectionsMarshal.GetValueRefOrNullRef(arbiters, kvArbiter.Key);
                arb.ApplyImpulse(spanBodies);
            }

            for (int j = 0; j < joints.Length; ++j)
            {
                joints[j].ApplyImpulse(spanBodies);
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

    void BroadPhase(Span<Body> spanBodies)
    {
        // O(n^2) broad-phase
        for (int i = 0; i < spanBodies.Length; ++i)
        {
            ref Body bi = ref spanBodies[i];

            for (int j = i + 1; j < spanBodies.Length; ++j)
            {
                ref Body bj = ref spanBodies[j];

                if (bi.invMass == 0.0f && bj.invMass == 0.0f)
                    continue;

                Arbiter newArb = new(spanBodies, new BodyIndex(i), new BodyIndex(j));
                ArbiterKey key = new(new BodyIndex(i), new BodyIndex(j));

                Program.DrawArbiterContacts(ref newArb);

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

    readonly Dictionary<ArbiterKey, Arbiter> arbiters;
    Vec2 gravity;
    int iterations;

    public static bool accumulateImpulses = true;
    public static bool warmStarting = true;
    public static bool positionCorrection = true;
};