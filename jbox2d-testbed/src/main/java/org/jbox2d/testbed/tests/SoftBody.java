package org.jbox2d.testbed.tests;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.testbed.framework.TestbedTest;

public class SoftBody extends TestbedTest {

	@Override
	public void initTest(boolean argDeserialized) {
		// TODO Auto-generated method stub
		Body ground = null;
		{
			BodyDef bd = new BodyDef();

			ground = getWorld().createBody(bd);

			PolygonShape shape = new PolygonShape();
			shape.setAsEdge(new Vec2(-400.0f, 0.0f), new Vec2(400.0f, 0.0f));

			FixtureDef fixtureDef = new FixtureDef();
			fixtureDef.friction = 1f;
			fixtureDef.shape = shape;
			ground.createFixture(fixtureDef);
		}

	
		
		{
			FixtureDef fixtureDef = new FixtureDef();
			fixtureDef.density = 2f;

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.position.set(0.0f, 30.0f);

			// create a long stick
			PolygonShape stickShape = new PolygonShape();
			stickShape.setAsBox(5f, 0.2f);

			fixtureDef.shape = stickShape;
			fixtureDef.friction = 1.0f;
			fixtureDef.restitution = 0.0f;
			Body stick= getWorld().createBody(bd);

			stick.createFixture(fixtureDef);

			// create a circle
			CircleShape circleShape = new CircleShape();
			circleShape.m_radius = 1.f;
			fixtureDef.shape = circleShape;
			fixtureDef.density = 1f;

			bd.position.set(10.f, 30.f);
			Body circle = getWorld().createBody(bd);
			circle.createFixture(fixtureDef);

			// createDistanceJoint(ground, circle, 1.0f, 0.0f, 10.f, 30.f, 0.0f,
			// 0.0f);

			// create a box

			PolygonShape boxShape = new PolygonShape();
			boxShape.setAsBox(3.f, 3.f);
			fixtureDef.shape = boxShape;
			fixtureDef.density = 0.5f;

			bd.position.set(15.f, 30.f);
			Body box = getWorld().createBody(bd);
			box.createFixture(fixtureDef);
			
			String path = 
		             
		             "195.00,122.00 146.00,111.00 146.00,111.00\n" +
		             "180.00,186.00 195.00,122.00 195.00,122.00\n" +
		             "144.00,183.00 180.00,186.00 180.00,186.00\n" +
		             "146.00,111.00 144.00,183.00 144.00,183.00\n" ;
			Scanner scanner = new Scanner(path);
			PolygonShape polygonShape = new PolygonShape();
			
			List<Vec2> vertices = new ArrayList<Vec2>();
			while (scanner.hasNextLine())
			{
				Vec2 vec = new Vec2();
				String[] pair = scanner.next().split(",");
				
				vec.x = Float.parseFloat(pair[0]);
				vec.y = Float.parseFloat(pair[1]);
				
				System.out.println(vec);
				scanner.nextLine();
				vertices.add(vec);
			}
			polygonShape.set(vertices.toArray(new Vec2[0]), vertices.size());
			fixtureDef.shape = polygonShape;
			
//			polygonShape.computeCentroidToOut(vertices.toArray(new Vec2[0]), vertices.size(), centroid);
			AABB aabb = new AABB();
			Transform transform = new Transform(new Vec2(0f, 0), new Mat22(1f, 0f, 0f, 1f));

			polygonShape.computeAABB(aabb, transform);
			
			AABB target = new AABB(new Vec2(0f, 0f), new Vec2(5f, 5f));
			Vec2 targetDimension = target.upperBound.sub(target.lowerBound);
			Vec2 srcDimension = aabb.upperBound.sub(aabb.lowerBound);
			
			float scaleX = targetDimension.x / srcDimension.x;
			float scaleY = targetDimension.y / srcDimension.y;
			
			for (Vec2 v : polygonShape.m_vertices)
			{
				v.x *= scaleX;
				v.y *= scaleY;
			}
				
			polygonShape.computeAABB(aabb, transform);
			Vec2 centroid = aabb.getCenter();
			System.out.println(aabb.getCenter());
			bd.position.set(-centroid.x, 0f);
			Body polygon = getWorld().createBody(bd);
			polygon.createFixture(fixtureDef);
			
			
		}

		final int nMasses = 100; // number of mass points

		final float width = 23f;

		final float frequencyHz = 5.0f;
		final float dampingRatio = 1.0f;

		{

			final float intervalSpace = width / (nMasses);
			final float massRadius = intervalSpace / 3;
			final float height = intervalSpace * 2;
			Body[] masses = new Body[nMasses];

			CircleShape shape = new CircleShape(); // create the shape
			shape.m_radius = massRadius;

			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.fixedRotation = true;

			float leftMostPos = -intervalSpace * (nMasses - 1) / 2;

			// create the masses
			for (int i = 0; i < masses.length; i++) {
				// position the mass
				float xPos = leftMostPos + i * intervalSpace;
				bd.position.set(xPos, height);

				// create the body
				masses[i] = getWorld().createBody(bd);

				// create the fixture
				FixtureDef fixtureDef = new FixtureDef();
				fixtureDef.friction = 1.0f;
				fixtureDef.shape = shape;
				fixtureDef.density = .2f;
				fixtureDef.restitution = 0.0f;
				masses[i].createFixture(fixtureDef);

			}
			// create the distant joints
			for (int i = 0; i < nMasses; i++) {
				// position the mass
				float xPos = leftMostPos + i * intervalSpace;

				createDistanceJoint(ground, masses[i], frequencyHz,
						dampingRatio, // vertical springs
						xPos);
				createDistanceJoint(ground, masses[i], frequencyHz,
						dampingRatio, xPos, height, 0.0f, 0.0f);

				if (i != 0) {
					createDistanceJoint(masses[i - 1], masses[i],
							frequencyHz * 3, dampingRatio); // horizontal
															// springs
					createDistanceJoint(ground, masses[i], frequencyHz * 2,
							dampingRatio, xPos - intervalSpace); // diagonal
																	// springs
				}

				if (i != nMasses - 1) {
					createDistanceJoint(ground, masses[i], frequencyHz * 2,
							dampingRatio, xPos + intervalSpace); // back
																	// diagonal
				}

			}

			//
			// jd.bodyA = ground;
			// jd.bodyB = body;
			// jd.localAnchorA.set(-0.f, .5f);
			// jd.localAnchorB.set(-0.f, .0f);
			// p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			// p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			// d = p2.sub(p1);
			// jd.length = d.length();
			// joint = getWorld().createJoint(jd);
			//
			// jd.bodyA = ground;
			// jd.bodyB = body;
			// jd.localAnchorA.set(5.f, .5f);
			// jd.localAnchorB.set(-0.f, 0.f);
			// p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
			// p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
			// d = p2.sub(p1);
			// jd.length = d.length();
			// System.out.println(jd.length);
			// joint = getWorld().createJoint(jd);
		}
	}

	private void createDistanceJoint(Body bodyA, Body bodyB,
			final float frequencyHz, final float dampingRatio) {
		createDistanceJoint(bodyA, bodyB, frequencyHz, dampingRatio, 0.0f);
	}

	private void createDistanceJoint(Body bodyA, Body bodyB,
			final float frequencyHz, final float dampingRatio, float xPosA) {
		createDistanceJoint(bodyA, bodyB, frequencyHz, dampingRatio, xPosA, 0f,
				0f, 0f);
	}

	private void createDistanceJoint(Body bodyA, Body bodyB,
			final float frequencyHz, final float dampingRatio, float xPosA,
			float yPosA, float xPosB, float yPosB) {
		DistanceJointDef jd = new DistanceJointDef();
		Vec2 p1 = new Vec2();
		Vec2 p2 = new Vec2();
		Vec2 d = new Vec2();

		jd.frequencyHz = frequencyHz;
		jd.dampingRatio = dampingRatio;
		jd.collideConnected = true;

		jd.bodyA = bodyA;
		jd.bodyB = bodyB;
		jd.localAnchorA.set(xPosA, yPosA); // local to bodyA
		jd.localAnchorB.set(xPosB, yPosB); // local to bodyB
		p1 = jd.bodyA.getWorldPoint(jd.localAnchorA);
		p2 = jd.bodyB.getWorldPoint(jd.localAnchorB);
		d = p2.sub(p1);
		jd.length = d.length();
		getWorld().createJoint(jd);
	}

	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Soft Body";
	}

}
