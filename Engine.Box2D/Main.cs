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

/*
 * Source from: https://github.com/erincatto/box2d-lite
 * Supplemental information: https://www.youtube.com/watch?v=0qqzGkbLbpM
 */

using System.Diagnostics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using ErrorCode = OpenTK.Graphics.OpenGL.ErrorCode;

namespace Engine.Box2D;

internal static class Program
{
    public static Body[] bodies = new Body[200];
    public static Joint[] joints = new Joint[100];
	
    static Body bomb = default;

    static float timeStep = 1.0f / 60.0f;
    static int iterations = 10;
    static Vec2 gravity = new(0.0f, -10.0f);

    public static int numBodies = 0;
    public static int numJoints = 0;

    static int demoIndex = 0;

    static World world = new(gravity, iterations);

    static void DrawText(int x, int y, string str)
    {
	    int len, i;

        /*
        GL.MatrixMode(MatrixMode.Projection);
        GL.PushMatrix();
        GL.LoadIdentity();
	    int w = glutGet(GLUT_WINDOW_WIDTH);
	    int h = glutGet(GLUT_WINDOW_HEIGHT);
	    gluOrtho2D(0, w, h, 0);
        GL.MatrixMode(MatrixMode.Modelview);
        GL.PushMatrix();
        GL.LoadIdentity();

        GL.Color3(0.9f, 0.6f, 0.6f);
        GL.RasterPos2(x, y);
	    len = (int) str.Length;
	    for (i = 0; i < len; i++)
		    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, str[i]);

        GL.PopMatrix();
        GL.MatrixMode(MatrixMode.Projection);
        GL.PopMatrix();
		*/
        GL.MatrixMode(MatrixMode.Modelview);
    }

    static void DrawBody(ref Body body)
    {
	    Mat22 R = new(body.rotation);
	    Vec2 x = body.position;
	    Vec2 h = 0.5f * body.width;

	    Vec2 v1 = x + R * new Vec2(-h.x, -h.y);
	    Vec2 v2 = x + R * new Vec2( h.x, -h.y);
	    Vec2 v3 = x + R * new Vec2( h.x,  h.y);
	    Vec2 v4 = x + R * new Vec2(-h.x,  h.y);
		
        GlCheckError();
	    if (body == bomb)
		    GL.Color3(0.4f, 0.9f, 0.4f);
	    else
		    GL.Color3(0.8f, 0.8f, 0.9f);
		
        GlCheckError();
        GL.Begin(PrimitiveType.LineLoop);
        GL.Vertex2(v1.x, v1.y);
        GL.Vertex2(v2.x, v2.y);
        GL.Vertex2(v3.x, v3.y);
        GL.Vertex2(v4.x, v4.y);
        GL.End();
        GlCheckError();

        Vec2 v = body.velocity;
		GL.Begin(PrimitiveType.Lines);

        GL.Color3(1f, 0.5f, 0.0f);
        GL.Vertex2(x.x, x.y);
        GL.Vertex2(x.x + v.x, x.y + v.y);

        GL.End();
    }

    static void DrawJoint(ref Joint joint)
    {
	    ref Body b1 = ref bodies[joint.body1Ref];
	    ref Body b2 = ref bodies[joint.body2Ref];

	    Mat22 R1 = new(b1.rotation);
	    Mat22 R2 = new(b2.rotation);

	    Vec2 x1 = b1.position;
	    Vec2 p1 = x1 + R1 * joint.localAnchor1;

	    Vec2 x2 = b2.position;
	    Vec2 p2 = x2 + R2 * joint.localAnchor2;
		
        GlCheckError();
        GL.Color3(0.5f, 0.5f, 0.8f);
        GlCheckError();
        GL.Begin(PrimitiveType.Lines);
        GL.Vertex2(x1.x, x1.y);
        GL.Vertex2(p1.x, p1.y);
        GL.Vertex2(x2.x, x2.y);
        GL.Vertex2(p2.x, p2.y);
        GL.End();
        GlCheckError();
    }

    public static void DrawArbiterContacts(ref Arbiter newArb)
    {
        GL.PointSize(4.0f);
        GL.Color3(1.0f, 0.0f, 0.0f);
        GL.Begin(PrimitiveType.Points);
        for (int i = 0; i < newArb.numContacts; ++i)
        {
            GL.Vertex2(newArb.contacts[i].position.x, newArb.contacts[i].position.y);
        }

        GL.End();
        GL.PointSize(1.0f);
    }

    static void LaunchBomb()
    {
	    if (bomb == default)
	    {
		    bomb = bodies[numBodies++];
		    bomb.Set(new Vec2(1.0f, 1.0f), 50.0f);
		    bomb.friction = 0.2f;
	    }

	    bomb.position.Set(FMath.Random(-15.0f, 15.0f), 15.0f);
	    bomb.rotation = FMath.Random(-1.5f, 1.5f);
	    bomb.velocity = -1.5f * bomb.position;
	    bomb.angularVelocity = FMath.Random(-20.0f, 20.0f);
        bodies[numBodies - 1] = bomb;
    }

    // Single box
    static void Demo1(Span<Body> sb, Span<Joint> sj)
    {
        ref Body b = ref sb[numBodies++];
        b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.position.Set(0.0f, -0.5f * b.width.y);

        b = ref sb[numBodies++];
        b.Set(new Vec2(1.0f, 1.0f), 200.0f);
	    b.position.Set(0.0f, 4.0f);
        b.rotation = -0.25f;
    }

    // A simple pendulum
    static void Demo2(Span<Body> sb, Span<Joint> sj)
    {
        ref Body b1 = ref sb[numBodies + 0];
        b1.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b1.friction = 0.2f;
	    b1.position.Set(0.0f, -0.5f * b1.width.y);
	    b1.rotation = 0.0f;

        ref Body b2 = ref sb[numBodies + 1];
	    b2.Set(new Vec2(1.0f, 1.0f), 100.0f);
	    b2.friction = 0.2f;
	    b2.position.Set(9.0f, 11.0f);
	    b2.rotation = 0.0f;

        ref Joint j = ref sj[numJoints++];
	    j.Set(sb, new BodyIndex(numBodies + 0), new BodyIndex(numBodies + 1), new Vec2(0.0f, 11.0f));

        numBodies += 2;
    }

    // Varying friction coefficients
    static void Demo3(Span<Body> sb, Span<Joint> sj)
    {
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.position.Set(0.0f, -0.5f * b.width.y);

        b = ref sb[numBodies++];
	    b.Set(new Vec2(13.0f, 0.25f), float.MaxValue);
	    b.position.Set(-2.0f, 11.0f);
	    b.rotation = -0.25f;

        b = ref sb[numBodies++];
	    b.Set(new Vec2(0.25f, 1.0f), float.MaxValue);
	    b.position.Set(5.25f, 9.5f);

        b = ref sb[numBodies++];
	    b.Set(new Vec2(13.0f, 0.25f), float.MaxValue);
	    b.position.Set(2.0f, 7.0f);
	    b.rotation = 0.25f;

        b = ref sb[numBodies++];
	    b.Set(new Vec2(0.25f, 1.0f), float.MaxValue);
	    b.position.Set(-5.25f, 5.5f);

        b = ref sb[numBodies++];
	    b.Set(new Vec2(13.0f, 0.25f), float.MaxValue);
	    b.position.Set(-2.0f, 3.0f);
	    b.rotation = -0.25f;

        //C# does not support const arrays
	    float[] friction = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
	    for (int i = 0; i < 5; ++i)
	    {
            b = ref sb[numBodies++];
		    b.Set(new Vec2(0.5f, 0.5f), 25.0f);
		    b.friction = friction[i];
		    b.position.Set(-7.5f + 2.0f * i, 14.0f);
        }
    }

    // A vertical stack
    static void Demo4(Span<Body> sb, Span<Joint> sj)
    {
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.friction = 0.2f;
	    b.position.Set(0.0f, -0.5f * b.width.y);
	    b.rotation = 0.0f;

        for (int i = 0; i < 10; ++i)
	    {
            b = ref sb[numBodies++];
		    b.Set(new Vec2(1.0f, 1.0f), 1.0f);
		    b.friction = 0.2f;
		    float x = FMath.Random(-0.1f, 0.1f);
		    b.position.Set(x, 0.51f + 1.05f * i);
        }
    }

    // A pyramid
    static void Demo5(Span<Body> sb, Span<Joint> sj)
    {
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.friction = 0.2f;
	    b.position.Set(0.0f, -0.5f * b.width.y);
	    b.rotation = 0.0f;

        Vec2 x = new(-6.0f, 0.75f);
	    Vec2 y;

	    for (int i = 0; i < 12; ++i)
	    {
		    y = x;

		    for (int j = i; j < 12; ++j)
		    {
                b = ref sb[numBodies++];
			    b.Set(new Vec2(1.0f, 1.0f), 10.0f);
			    b.friction = 0.2f;
			    b.position = y;

                y += new Vec2(1.125f, 0.0f);
		    }

		    //x += Vec2(0.5625f, 1.125f);
		    x += new Vec2(0.5625f, 2.0f);
	    }
    }

    // A teeter
    static void Demo6(Span<Body> sb, Span<Joint> sj)
    {
	    ref Body b1 = ref sb[numBodies + 0];
	    b1.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b1.position.Set(0.0f, -0.5f * b1.width.y);

        ref Body b2 = ref sb[numBodies + 1];
	    b2.Set(new Vec2(12.0f, 0.25f), 100.0f);
	    b2.position.Set(0.0f, 1.0f);

        ref Body b3 = ref sb[numBodies + 2];
	    b3.Set(new Vec2(0.5f, 0.5f), 25.0f);
	    b3.position.Set(-5.0f, 2.0f);

        ref Body b4 = ref sb[numBodies + 3];
	    b4.Set(new Vec2(0.5f, 0.5f), 25.0f);
	    b4.position.Set(-5.5f, 2.0f);

        ref Body b5 = ref sb[numBodies + 4];
	    b5.Set(new Vec2(1.0f, 1.0f), 100.0f);
	    b5.position.Set(5.5f, 15.0f);

        ref Joint j = ref sj[numJoints++];
	    j.Set(sb, new BodyIndex(numBodies + 0), new BodyIndex(numBodies + 1), new Vec2(0.0f, 1.0f));

        numBodies += 5;
    }

    // A suspension bridge
    static void Demo7(Span<Body> sb, Span<Joint> sj)
    {
        int baseRef = numBodies;
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.friction = 0.2f;
	    b.position.Set(0.0f, -0.5f * b.width.y);
	    b.rotation = 0.0f;

        const int numPlanks = 15;
	    float mass = 50.0f;

	    for (int i = 0; i < numPlanks; ++i)
	    {
            b = ref sb[numBodies++];
		    b.Set(new Vec2(1.0f, 0.25f), mass);
		    b.friction = 0.2f;
		    b.position.Set(-8.5f + 1.25f * i, 5.0f);
        }

	    // Tuning
	    float frequencyHz = 2.0f;
	    float dampingRatio = 0.7f;

	    // frequency in radians
	    float omega = 2.0f * MathF.PI * frequencyHz;

	    // damping coefficient
	    float d = 2.0f * mass * dampingRatio * omega;

	    // spring stifness
	    float k = mass * omega * omega;

	    // magic formulas
	    float softness = 1.0f / (d + timeStep * k);
	    float biasFactor = timeStep * k / (d + timeStep * k);
		    
	    for (int i = 0; i < numPlanks; ++i)
	    {
            ref Joint j = ref sj[numJoints++];
		    j.Set(sb, new BodyIndex(baseRef+i), new BodyIndex(baseRef+i+1), new Vec2(-9.125f + 1.25f * i, 5.0f));
		    j.softness = softness;
		    j.biasFactor = biasFactor;
        }
	    
        ref Joint j1 = ref sj[numJoints++];
	    j1.Set(sb, new BodyIndex(baseRef + numPlanks), new BodyIndex(baseRef), new Vec2(-9.125f + 1.25f * numPlanks, 5.0f));
	    j1.softness = softness;
	    j1.biasFactor = biasFactor;
    }

    // Dominos
    static void Demo8(Span<Body> sb, Span<Joint> sj)
    {
	    BodyIndex b1 = new BodyIndex(numBodies);
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.position.Set(0.0f, -0.5f * b.width.y);

        b = ref sb[numBodies++];
	    b.Set(new Vec2(12.0f, 0.5f), float.MaxValue);
	    b.position.Set(-1.5f, 10.0f);

        for (int i = 0; i < 10; ++i)
	    {
            b = ref sb[numBodies++];
		    b.Set(new Vec2(0.2f, 2.0f), 10.0f);
		    b.position.Set(-6.0f + 1.0f * i, 11.125f);
		    b.friction = 0.1f;
        }
	    
        b = ref sb[numBodies++];
	    b.Set(new Vec2(14.0f, 0.5f), float.MaxValue);
	    b.position.Set(1.0f, 6.0f);
	    b.rotation = 0.3f;

        BodyIndex b2 = new BodyIndex(numBodies);
        b = ref sb[numBodies++];
	    b.Set(new Vec2(0.5f, 3.0f), float.MaxValue);
	    b.position.Set(-7.0f, 4.0f);

        BodyIndex b3 = new BodyIndex(numBodies);
        b = ref sb[numBodies++];
	    b.Set(new Vec2(12.0f, 0.25f), 20.0f);
	    b.position.Set(-0.9f, 1.0f);

        ref Joint j = ref sj[numJoints++];
	    j.Set(sb, b1, b3, new Vec2(-2.0f, 1.0f));

        BodyIndex b4 = new BodyIndex(numBodies);
        b = ref sb[numBodies++];
	    b.Set(new Vec2(0.5f, 0.5f), 10.0f);
	    b.position.Set(-10.0f, 15.0f);

        j = ref sj[numJoints++];
	    j.Set(sb, b2, b4, new Vec2(-7.0f, 15.0f));

        BodyIndex b5 = new BodyIndex(numBodies);
        b = ref sb[numBodies++];
	    b.Set(new Vec2(2.0f, 2.0f), 20.0f);
	    b.position.Set(6.0f, 2.5f);
	    b.friction = 0.1f;

        j = ref sj[numJoints++];
	    j.Set(sb, b1, b5, new Vec2(6.0f, 2.6f));

        BodyIndex b6 = new BodyIndex(numBodies);
        b = ref sb[numBodies++];
	    b.Set(new Vec2(2.0f, 0.2f), 10.0f);
	    b.position.Set(6.0f, 3.6f);

        j = ref sj[numJoints++];
	    j.Set(sb, b5, b6, new Vec2(7.0f, 3.5f));
    }

    // A multi-pendulum
    static void Demo9(Span<Body> sb, Span<Joint> sj)
    {
        BodyIndex b1 = new BodyIndex(numBodies);
        ref Body b = ref sb[numBodies++];
	    b.Set(new Vec2(100.0f, 20.0f), float.MaxValue);
	    b.friction = 0.2f;
	    b.position.Set(0.0f, -0.5f * b.width.y);
	    b.rotation = 0.0f;

        float mass = 10.0f;

	    // Tuning
	    float frequencyHz = 4.0f;
	    float dampingRatio = 0.7f;

	    // frequency in radians
	    float omega = 2.0f * MathF.PI * frequencyHz;

	    // damping coefficient
	    float d = 2.0f * mass * dampingRatio * omega;

	    // spring stiffness
	    float k = mass * omega * omega;

	    // magic formulas
	    float softness = 1.0f / (d + timeStep * k);
	    float biasFactor = timeStep * k / (d + timeStep * k);

	    const float y = 12.0f;

	    for (int i = 0; i < 15; ++i)
	    {
		    Vec2 x = new(0.5f + i, y);

            b = ref sb[numBodies];
		    b.Set(new Vec2(0.75f, 0.25f), mass);
		    b.friction = 0.2f;
		    b.position = x;
		    b.rotation = 0.0f;

            ref Joint j = ref sj[numJoints];
		    j.Set(sb, b1, new BodyIndex(numBodies), new Vec2(i, y));
		    j.softness = softness;
		    j.biasFactor = biasFactor;

            b1 = new BodyIndex(numBodies);
		    ++numBodies;
		    ++numJoints;
	    }
    }

    delegate void DemoDelegate(Span<Body> sb, Span<Joint> sj);

    static readonly DemoDelegate[] demos = {Demo1, Demo2, Demo3, Demo4, Demo5, Demo6, Demo7, Demo8, Demo9};
	
    static readonly string[] demoStrings = {
	    "Demo 1: A Single Box",
	    "Demo 2: Simple Pendulum",
	    "Demo 3: Varying Friction Coefficients",
	    "Demo 4: Randomized Stacking",
	    "Demo 5: Pyramid Stacking",
	    "Demo 6: A Teeter",
	    "Demo 7: A Suspension Bridge",
	    "Demo 8: Dominos",
	    "Demo 9: Multi-pendulum"};

    static void InitDemo(int index)
    {
	    world.Clear();
	    numBodies = 0;
	    numJoints = 0;
	    bomb = default;

	    demoIndex = index;
	    demos[index](bodies, joints);
    }

    static void SimulationLoop(IGLFWGraphicsContext context)
    {
        GlCheckError();
	    GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
        GlCheckError();

	    DrawText(5, 15, demoStrings[demoIndex]);
        GlCheckError();
	    DrawText(5, 45, "Keys: 1-9 Demos, Space to Launch the Bomb");
        GlCheckError();

	    string buffer;
	    buffer = string.Format("(A)ccumulation {0}", World.accumulateImpulses ? "ON" : "OFF");
	    DrawText(5, 75, buffer);
        GlCheckError();

        buffer = string.Format("(P)osition Correction {0}", World.positionCorrection ? "ON" : "OFF");
	    DrawText(5, 105, buffer);
        GlCheckError();

        buffer = string.Format("(W)arm Starting {0}", World.warmStarting ? "ON" : "OFF");
	    DrawText(5, 135, buffer);
        GlCheckError();

	    GL.MatrixMode(MatrixMode.Modelview);
        GlCheckError();
        GL.LoadIdentity();
        GlCheckError();
        GL.Translate(0.0f, -7.0f, -25.0f);
        GlCheckError();

	    world.Step(timeStep);
        GlCheckError();

	    for (int i = 0; i < numBodies; ++i)
            DrawBody(ref bodies[i]);

        for (int i = 0; i < numJoints; ++i)
            DrawJoint(ref joints[i]);

		context.SwapBuffers();
        GlCheckError();
    }

    static void Keyboard(Keys key)
    {
	    switch (key)
	    {
	    case Keys.Escape:
		    Environment.Exit(0);
		    break;

	    case Keys.D1:
	    case Keys.D2:
	    case Keys.D3:
	    case Keys.D4:
	    case Keys.D5:
	    case Keys.D6:
	    case Keys.D7:
	    case Keys.D8:
	    case Keys.D9:
		    InitDemo(key - Keys.D1);
		    break;

	    case Keys.A:
		    World.accumulateImpulses = !World.accumulateImpulses;
		    break;

	    case Keys.P:
		    World.positionCorrection = !World.positionCorrection;
		    break;

	    case Keys.W:
		    World.warmStarting = !World.warmStarting;
		    break;

	    case Keys.Space:
		    LaunchBomb();
		    break;
	    }
    }

    static void Reshape(int width, int height)
    {
	    if (height == 0)
		    height = 1;

	    GL.Viewport(0, 0, width, height);
        GlCheckError();
        GL.MatrixMode(MatrixMode.Projection);
        GlCheckError();
        GL.LoadIdentity();
        GlCheckError();
	    gluPerspective(45.0, (float)width/(float)height, 0.1, 100.0);
        GlCheckError();
    }

    static void gluPerspective(double fovY, double aspectRatio, double front, double back)
    {
        const double DEG2RAD = 3.14159265 / 180;

        double tangent = Math.Tan(fovY/2 * DEG2RAD);   // tangent of half fovY
        double height = front * tangent;          // half height of near plane
        double width = height * aspectRatio;      // half width of near plane

        // params: left, right, bottom, top, near, far
        GL.Frustum(-width, width, -height, height, front, back);
    }

    private static int GlThread;
	[Conditional("DEBUG")]
    static void GlCheckError()
    {
		Debug.Assert(GlThread == Thread.CurrentThread.ManagedThreadId);

        ErrorCode err;
        while((err = GL.GetError()) != ErrorCode.NoError)
        {
            // Process/log the error.
			Debugger.Break();
        }
    }

    static int Main(string[] args)
    {
        InitDemo(0);

        // glutInit(&argc, argv);
        // glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
        // glutInitWindowSize(800, 800);
        // glutCreateWindow("Box2D");

        // glutReshapeFunc(Reshape);
        // glutDisplayFunc(SimulationLoop);
        // glutKeyboardFunc(Keyboard);
        // glutIdleFunc(SimulationLoop);
        //
        // glutMainLoop();

        var gameWindowSettings = new GameWindowSettings()
        {
            IsMultiThreaded = false
        };

        var wndSettings = new NativeWindowSettings()
        {
            Size = new Vector2i(800, 600),
			Title = "Box2D",
            //APIVersion = new Version(2, 0),
            Profile = ContextProfile.Compatability,
			Flags = ContextFlags.Debug
        };

        GlThread = Thread.CurrentThread.ManagedThreadId;
        using (var game = new Game(gameWindowSettings, wndSettings))
        {
            game.VSync = VSyncMode.On; // sim is bound to fps

            game.Resize += eventArgs => Reshape(eventArgs.Width, eventArgs.Height);
            game.RenderFrame += eventArgs => SimulationLoop(game.Context);
            game.KeyDown += eventArgs => Keyboard(eventArgs.Key);

			game.Run();
        }

        return 0;
    }
}

class Game : GameWindow
{
    public Game(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings) : base(gameWindowSettings, nativeWindowSettings) { }
}
