using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NUnit.Framework;

namespace Engine.Box2D.Tests
{
    [TestFixture]
    public class CollideTests
    {
        [Test]
        public void TwoOverlappingBoxes()
        {
            var bodies = new Body[2];

            ref var b1 = ref bodies[0];
            b1.Set(new Vec2(4, 4), 16);
            b1.position.Set(2, 2);

            ref var b2 = ref bodies[1];
            b2.Set(new Vec2(2, 2), 4);
            b2.position.Set(4.90f, 2);

            Arbiter newArb = new(new Span<Body>(bodies), new BodyIndex(0), new BodyIndex(1));
        }
    }
}
