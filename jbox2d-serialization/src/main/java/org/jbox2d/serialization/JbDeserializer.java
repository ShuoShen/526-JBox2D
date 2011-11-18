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

import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;

public interface JbDeserializer {
	
	/**
	 * Sets the object listener, which allows the
	 * user to process each physics object with a
	 * tag to do any sort of custom logic.
	 * @param argListener
	 */
	public void setObjectListener(ObjectListener argListener);
	
	/**
	 * Sets a listener for unsupported exceptions instead of 
	 * stopping the whole deserialization process by throwing
	 * and exception.
	 * @param argListener
	 */
	public void setUnsupportedListener(UnsupportedListener argListener);
	
	/**
	 * Deserializes a world
	 * @param argInput
	 * @return
	 * @throws IOException
	 * @throws UnsupportedObjectException if a read physics object is unsupported by this library
	 * @see #setUnsupportedListener(UnsupportedListener)
	 */
	public World deserializeWorld(InputStream argInput) throws IOException, UnsupportedObjectException;

	/**
   * Deserializes a body
   * @param argWorld
   * @param argInput
   * @return
   * @throws IOException
   * @throws UnsupportedObjectException if a read physics object is unsupported by this library
   * @see #setUnsupportedListener(UnsupportedListener)
   */
	public Body deserializeBody(World argWorld, InputStream argInput) throws IOException, UnsupportedObjectException;
	
	/**
   * Deserializes a fixture
   * @param argBody
   * @param argInput
   * @return
   * @throws IOException
   * @throws UnsupportedObjectException if a read physics object is unsupported by this library
   * @see #setUnsupportedListener(UnsupportedListener)
   */
	public Fixture deserializeFixture(Body argBody, InputStream argInput) throws IOException, UnsupportedObjectException;
	
	/**
   * Deserializes a shape
   * @param argInput
   * @return
   * @throws IOException
   * @throws UnsupportedObjectException if a read physics object is unsupported by this library
   * @see #setUnsupportedListener(UnsupportedListener)
   */
	public Shape deserializeShape(InputStream argInput) throws IOException, UnsupportedObjectException;
	
	/**
   * Deserializes a joint
   * @param argWorld
   * @param argInput
   * @param argBodyMap
   * @param argJointMap
   * @return
   * @throws IOException
   * @throws UnsupportedObjectException if a read physics object is unsupported by this library
   * @see #setUnsupportedListener(UnsupportedListener)
   */
	public Joint deserializeJoint(World argWorld, InputStream argInput, Map<Integer, 
								  Body> argBodyMap, Map<Integer, Joint> argJointMap) throws IOException, UnsupportedObjectException;
	
	/**
	 * Called for each physics object with a tag defined.
	 * @author dmurph
	 *
	 */
	public static interface ObjectListener{
		
		public void processWorld(World argWorld, Long argTag);
		
		public void processBody(Body argBody, Long argTag);
		
		public void processFixture(Fixture argFixture, Long argTag);
		
		public void processShape(Shape argShape, Long argTag);
		
		public void processJoint(Joint argJoint, Long argTag);
	}
}
