"use client";
import { Canvas, useFrame } from "@react-three/fiber";
import { Physics, RigidBody } from "@react-three/rapier";
import { OrbitControls } from "@react-three/drei";
import { useRef, useState } from "react";

// Simple Inverse Kinematics Solver
const solveIK = (targetPosition, baseHeight) => {
  const { x, y, z } = targetPosition;
  const distance = Math.sqrt(x * x + (y - baseHeight) * (y - baseHeight) + z * z);

  const joint1 = Math.atan2(z, x);
  const joint2 = -Math.PI / 4; // Simplified for now
  const joint3 = Math.asin((y - baseHeight) / distance);

  return { joint1, joint2, joint3 };
};

// Robot Component (Base + Arm)
const Robot = ({ position, target, onReachedTarget }) => {
  const baseRef = useRef();
  const joint1Ref = useRef();
  const joint2Ref = useRef();
  const joint3Ref = useRef();

  const [angles, setAngles] = useState({ joint1: 0, joint2: 0, joint3: 0 });
  const [isAtTarget, setIsAtTarget] = useState(false);

  // Move base towards target
  useFrame(() => {
    if (target) {
      const { x: tx, z: tz } = target;

      // Get current position
      const { x, z } = baseRef.current.translation();

      // Move towards target
      const dx = tx - x;
      const dz = tz - z;
      const distance = Math.sqrt(dx * dx + dz * dz);

      if (distance > 0.1) {
        baseRef.current.applyImpulse({ x: dx * 0.5, y: 0, z: dz * 0.5 }, true);
      } else if (!isAtTarget) {
        setIsAtTarget(true);
        onReachedTarget();
      }
    }
  });

  // Update IK when at target
  useFrame(() => {
    if (isAtTarget && target) {
      const newAngles = solveIK({ x: 0, y: 2, z: 0 }, 1);
      setAngles(newAngles);
    }
  });

  return (
    <RigidBody ref={baseRef} colliders="cuboid" position={position}>
      {/* Base */}
      <mesh>
        <boxGeometry args={[2, 0.5, 2]} />
        <meshStandardMaterial color="blue" />
      </mesh>

      {/* Wheels */}
      {[[-1, -0.25, -1], [1, -0.25, -1], [-1, -0.25, 1], [1, -0.25, 1]].map(
        ([x, y, z], i) => (
          <mesh key={i} position={[x, y, z]} rotation={[0, 0, Math.PI / 2]}>
            <cylinderGeometry args={[0.3, 0.3, 0.2, 16]} />
            <meshStandardMaterial color="black" />
          </mesh>
        )
      )}

      {/* Robotic Arm */}
      <group position={[0, 0.25, 0]}>
        {/* Joint 1 */}
        <group ref={joint1Ref} rotation={[0, angles.joint1, 0]}>
          <mesh position={[0, 1, 0]}>
            <boxGeometry args={[0.3, 2, 0.3]} />
            <meshStandardMaterial color="red" />
          </mesh>

          {/* Joint 2 */}
          <group ref={joint2Ref} position={[0, 2, 0]} rotation={[angles.joint2, 0, 0]}>
            <mesh position={[0, 1, 0]}>
              <boxGeometry args={[0.3, 2, 0.3]} />
              <meshStandardMaterial color="green" />
            </mesh>

            {/* Joint 3 */}
            <group ref={joint3Ref} position={[0, 2, 0]} rotation={[angles.joint3, 0, 0]}>
              <mesh position={[0, 1, 0]}>
                <boxGeometry args={[0.3, 2, 0.3]} />
                <meshStandardMaterial color="blue" />
              </mesh>
            </group>
          </group>
        </group>
      </group>
    </RigidBody>
  );
};

// Environment with Blocks
const Environment = ({ onBlockSelect }) => {
  const blocks = [
    { id: 1, position: [3, 0.5, -2] },
    { id: 2, position: [-3, 0.5, 2] },
  ];

  return (
    <>
      {/* Ground */}
      <RigidBody type="fixed" colliders="cuboid">
        <mesh position={[0, -0.5, 0]}>
          <boxGeometry args={[20, 1, 20]} />
          <meshStandardMaterial color="lightgray" />
        </mesh>
      </RigidBody>

      {/* Blocks */}
      {blocks.map(({ id, position }) => (
        <RigidBody key={id} colliders="cuboid" position={position}>
          <mesh
            onClick={() => onBlockSelect(position)}
            position={[0, 0.5, 0]}
          >
            <boxGeometry args={[1, 1, 1]} />
            <meshStandardMaterial color="orange" />
          </mesh>
        </RigidBody>
      ))}
    </>
  );
};

// Main Experience
const Experience = () => {
  const [target, setTarget] = useState(null);

  const handleBlockSelect = (position) => {
    setTarget({ x: position[0], z: position[2] });
  };

  const handleTargetReached = () => {
    console.log("Robot has reached the target and arm is ready.");
  };

  return (
    <Canvas shadows camera={{ position: [10, 10, 10], fov: 50 }}>
      <ambientLight intensity={0.5} />
      <directionalLight position={[5, 10, 5]} castShadow />

      <Physics gravity={[0, -9.81, 0]}>
        <Robot position={[0, 0.5, 0]} target={target} onReachedTarget={handleTargetReached} />
        <Environment onBlockSelect={handleBlockSelect} />
      </Physics>

      <OrbitControls />
    </Canvas>
  );
};

export default Experience;
