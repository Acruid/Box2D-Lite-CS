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

using System.Diagnostics;

namespace Engine.Box2D;

public static class FMath
{
    public const float k_pi = 3.14159265358979323846264f;

    // https://docs.microsoft.com/en-us/dotnet/api/system.random.next?view=net-6.0
    public const int RAND_MAX = int.MaxValue - 1;

    private static readonly Random rng = new Random();

    public static float Abs(float a)
    {
        return a > 0.0f ? a : -a;
    }
    public static float Sign(float x)
    {
        return x < 0.0f ? -1.0f : 1.0f;
    }

    public static float Min(float a, float b)
    {
        return a < b ? a : b;
    }

    public static float Max(float a, float b)
    {
        return a > b ? a : b;
    }

    public static float Clamp(float a, float low, float high)
    {
        return Max(low, Min(a, high));
    }

    public static void Swap<T>(ref T a, ref T b)
    {
        T tmp = a;
        a = b;
        b = tmp;
    }

    // Random number in range [-1,1]
    public static float Random()
    {
        float r = rng.Next();
        r /= RAND_MAX;
        r = 2.0f * r - 1.0f;
        return r;
    }

    public static float Random(float lo, float hi)
    {
        float r = rng.Next();
        r /= RAND_MAX;
        r = (hi - lo) * r + lo;
        return r;
    }
}


public struct Vec2 : IEquatable<Vec2>
{
    public Vec2(float x, float y)
    {
        this.x = x;
        this.y = y;
    }

    public void Set(float x_, float y_) { x = x_; y = y_; }

	public static Vec2 operator -(Vec2 a) { return new Vec2(-a.x, -a.y); }

    public readonly float Length()
	{
		return MathF.Sqrt(x * x + y * y);
	}

    public float x;
    public float y;

    public static float Dot(in Vec2 a, in Vec2 b)
    {
        return a.x * b.x + a.y * b.y;
    }

    public static float Cross(in Vec2 a, in Vec2 b)
    {
        return a.x * b.y - a.y * b.x;
    }

    public static Vec2 Cross(in Vec2 a, float s)
    {
        return new Vec2(s * a.y, -s * a.x);
    }

    public static Vec2 Cross(float s, in Vec2 a)
    {
        return new Vec2(-s * a.y, s * a.x);
    }

    public static Vec2 operator + (in Vec2 a, in Vec2 b)
    {
        return new Vec2(a.x + b.x, a.y + b.y);
    }

    public static  Vec2 operator - (in Vec2 a, in Vec2 b)
    {
        return new Vec2(a.x - b.x, a.y - b.y);
    }

    public static Vec2 operator * (float s, in Vec2 v)
    {
        return new Vec2(s * v.x, s * v.y);
    }

    public static Vec2 Abs(in Vec2 a)
    {
        return new Vec2(MathF.Abs(a.x), MathF.Abs(a.y));
    }

    #region Equality

    public bool Equals(Vec2 other)
    {
        return x.Equals(other.x) && y.Equals(other.y);
    }

    public override bool Equals(object? obj)
    {
        return obj is Vec2 other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(x, y);
    }

    public static bool operator ==(Vec2 left, Vec2 right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(Vec2 left, Vec2 right)
    {
        return !left.Equals(right);
    }

    #endregion
}

public struct Mat22
{
	public Mat22(float angle)
    {
        col1 = default;
        col2 = default;

		float c = MathF.Cos(angle), s = MathF.Sin(angle);
        col1.x = c; col2.x = -s;
        col1.y = s; col2.y = c;
	}

    public Mat22(in Vec2 col1, in Vec2 col2)
    {
        this.col1 = col1;
        this.col2 = col2;
    }

	public readonly Mat22 Transpose()
	{
		return new Mat22(new Vec2(col1.x, col2.x), new Vec2(col1.y, col2.y));
	}

	public readonly Mat22 Invert()
	{
		float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		Mat22 B;
		float det = a * d - b * c;
		Debug.Assert(det != 0.0f);
		det = 1.0f / det;
		B.col1.x =  det * d;	B.col2.x = -det * b;
		B.col1.y = -det * c;	B.col2.y =  det * a;
		return B;
	}

	public Vec2 col1, col2;

    public static Vec2 operator * (in Mat22 A, in Vec2 v)
    {
        return new Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
    }

    public static Mat22 operator + (in Mat22 A, in Mat22 B)
    {
        return new Mat22(A.col1 + B.col1, A.col2 + B.col2);
    }

    public static Mat22 operator * (in Mat22 A, in Mat22 B)
    {
        return new Mat22(A * B.col1, A * B.col2);
    }

    public static Mat22 Abs(in Mat22 A)
    {
        return new Mat22(Vec2.Abs(A.col1), Vec2.Abs(A.col2));
    }
};
