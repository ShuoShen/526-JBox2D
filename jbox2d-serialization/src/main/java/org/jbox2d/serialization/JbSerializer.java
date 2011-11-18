/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.serialization;

import java.util.Map;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;

/**
 * Serializer for jbox2d, used to 
 * serialize any aspect of the physics world
 * @author Daniel
 *
 */
public interface JbSerializer {
	
	/**
	 * Sets the object signer for the serializer.  This allows
	 * the user to specify an 'tag' for each main physics object,
	 * which is then referenced later at deserialization for the user.
	 * @param argSigner
	 */
	public void setObjectSigner(ObjectSigner argSigner);
	
	/**
	 * Sets a listener for unsupported exception instead of 
	 * stopping the whole serialization process by throwing
	 * and exception.
	 * @param argListener
	 */
	public void setUnsupportedListener(UnsupportedListener argListener);

	/**
	 * Serializes the world
	 * @param argWorld
	 * @return
   * @throws UnsupportedObjectException if a physics object is unsupported by this library.
   * @see #setUnsupportedListener(UnsupportedListener)
	 */
	public SerializationResult serialize(World argWorld) throws UnsupportedObjectException;
	
	/**
	 * Serializes a body
	 * @param argBody
	 * @return
   * @throws UnsupportedObjectException if a physics object is unsupported by this library.
   * @see #setUnsupportedListener(UnsupportedListener)
	 */
	public SerializationResult serialize(Body argBody) throws UnsupportedObjectException;
	
	/**
	 * Serializes a fixture
	 * @param argFixture
	 * @return
	 * @throws UnsupportedObjectException if a physics object is unsupported by this library.
	 * @see #setUnsupportedListener(UnsupportedListener)
	 */
	public SerializationResult serialize(Fixture argFixture) throws UnsupportedObjectException;
	
	/**
	 * Serializes a shape
	 * @param argShape
	 * @return
   * @throws UnsupportedObjectException if a physics object is unsupported by this library.
   * @see #setUnsupportedListener(UnsupportedListener)
	 */
	public SerializationResult serialize(Shape argShape) throws UnsupportedObjectException;
	
	/**
	 * Serializes joints.  Joints need to reference bodies
	 * and sometimes other joints.
	 * @param argJoint
	 * @param argBodyIndexMap
	 * @param argJointIndexMap
	 * @return
	 */
	public SerializationResult serialize(Joint argJoint,
			Map<Body, Integer> argBodyIndexMap,
			Map<Joint, Integer> argJointIndexMap);
	
	/**
	 * Interface that allows the serializer to
	 * look up tags for each object, which can be
	 * used later during deserializing by the developer.
	 * @author Daniel
	 */
	public static interface ObjectSigner {
		/**
		 * @param argWorld
		 * @return the tag for the world. can be null.
		 */
		public Long getTag(World argWorld);
		/**
		 * @param argBody
		 * @return the tag for the body.  can be null.
		 */
		public Long getTag(Body argBody);
		/**
		 * @param argShape
		 * @return the tag for the shape. can be null.
		 */
		public Long getTag(Shape argShape);
		/**
		 * @param argFixture
		 * @return the tag for the fixture. can be null.
		 */
		public Long getTag(Fixture argFixture);
		/**
		 * @param argJoint
		 * @return the tag for the joint. can be null.
		 */
		public Long getTag(Joint argJoint);
	}
}
