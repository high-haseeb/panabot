"use client";
import { Canvas, useFrame } from "@react-three/fiber";
import { Physics, RigidBody } from "@react-three/rapier";
import { OrbitControls } from "@react-three/drei";
import { useRef, useState, useEffect } from "react";
import { Vector3 } from "three";
import * as THREE from "three";

const solveIK = (targetPosition, targetOrientation, baseHeight, armLengths) => {
    const { x, y, z } = targetPosition;
    const { roll, pitch, yaw } = targetOrientation;
    const { l1, l2 } = armLengths;

    // Calculate wrist position (subtract base height from y)
    const wristX = x;
    const wristY = y - baseHeight;
    const wristZ = z;

    // Step 1: Calculate Joint 1 (Base rotation, around Z-axis)
    const joint1 = Math.atan2(wristY, wristX); // In 2D, this is atan2(y, x)

    // Step 2: Compute distance from base to wrist (r) and the vertical component (s)
    const r = Math.sqrt(wristX ** 2 + wristY ** 2); // 2D distance (horizontal)
    const s = wristZ; // Vertical distance (along Z-axis)

    // Step 3: Using the law of cosines to calculate joint 3 (elbow angle)
    const D = (r ** 2 + s ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2);

    if (D < -1 || D > 1) {
        console.warn("Target is out of reach");
        return { joint1: 0, joint2: 0, joint3: 0, wristRoll: 0, wristPitch: 0, wristYaw: 0 }; // Target unreachable
    }

    // Step 4: Compute Joint 3 (Elbow angle)
    const joint3 = Math.acos(D);

    // Step 5: Calculate Joint 2 (Shoulder pitch angle)
    const theta1 = Math.atan2(s, r); // Angle between arm and Z-axis
    const theta2 = Math.atan2(l2 * Math.sin(joint3), l1 + l2 * Math.cos(joint3));
    const joint2 = theta1 - theta2;

    // Step 6: Compute Wrist Angles (Direct from target orientation)
    const wristRoll = roll; // Rotation about the forearm axis
    const wristPitch = pitch; // Up and down rotation (forearm pitch)
    const wristYaw = yaw; // Side-to-side rotation (forearm yaw)

    // Return all calculated joint angles
    return {
        joint1,
        joint2,
        joint3,
        wristRoll,
        wristPitch,
        wristYaw,
    };
};

const RobotArm = (target) => {

    const { endEffectorX, endEffectorY, endEffectorZ } = target;
    // const { endEffectorX, endEffectorY, endEffectorZ } = useControls({
    //   endEffectorX: { value: 1, min: -5, max: 5 },
    //   endEffectorY: { value: 1, min: -5, max: 5 },
    //   endEffectorZ: { value: 0, min: -5, max: 5 },
    // });

    const [computedAngles, setComputedAngles] = useState({ angle1: 0, angle2: 0, angle3: 0 });
    // const { setAngles, setOutOfReach } = useRobotStore();
    const [angles, setAngles] = useState({});
    const [outOfReach, setOutOfReach] = useState(false);

    useEffect(() => {
        const L1 = 1;
        const L2 = 1;
        const L3 = 1;

        const target = new Vector3(endEffectorX, endEffectorY, endEffectorZ);
        const x = target.y;
        const y = target.x;
        const z = target.z;

        const r = Math.sqrt(x * x + y * y);
        const d = Math.sqrt(r * r + z * z);
        if (d > L1 + L2 + L3 || d < Math.abs(L1 - L2 - L3)) {
            console.error("Target is out of reach");
            setOutOfReach(true);
            return;
        } else {
            setOutOfReach(false);
        }
        const theta1 = Math.atan2(y, x);
        const r2 = r - L1;
        const cosTheta3 = (r2 * r2 + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        const theta3 = Math.atan2(Math.sqrt(1 - cosTheta3 * cosTheta3), cosTheta3);
        const k1 = L2 + L3 * Math.cos(theta3);
        const k2 = L3 * Math.sin(theta3);
        const theta2 = Math.atan2(z, r2) - Math.atan2(k2, k1);

        setAngles([theta1.toFixed(2), theta2.toFixed(2), theta3.toFixed(2)]);
        setComputedAngles({ angle1: theta1, angle2: theta2, angle3: theta3 });
    }, [endEffectorX, endEffectorY, endEffectorZ]);

    return (
        <group position={[0, 1.0, 0]}>
            <group position={[0, 0, 0]} rotation={[0, computedAngles.angle1, 0]}>
                <Solid rotation={[0, 0, 0]} color="orange" />
                <Link start={new Vector3(0, 0, 0)} end={new Vector3(0, 1, 0)} />
                <group position={[0, 1, 0]} rotation={[0, 0, computedAngles.angle2]}>
                    <Solid rotation={[Math.PI / 2, 0, 0]} color="lime" />
                    <Link start={new Vector3(0, 0, 0.7)} end={new Vector3(0, 2, 0.7)} />
                    <group position={[0, 2, 0]} rotation={[0, 0, computedAngles.angle3]}>
                        <Solid rotation={[Math.PI / 2, 0, 0]} color="brown" />
                        <Link start={new Vector3(0, 0, 0)} end={new Vector3(-1, 2, 0)} log />
                    </group>
                </group>
            </group>
        </group>
    );
};
const Link = ({ start, end, log = false }) => {
    const ref = useRef(null);

    useEffect(() => {
        if (ref.current) {
            const length = start.distanceTo(end);
            ref.current.scale.set(1, length, 1);

            const middle = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
            ref.current.position.set(middle.x, middle.y, middle.z);

            const direction = new THREE.Vector3().subVectors(end, start).normalize();
            const up = new THREE.Vector3(0, 1, 0);
            const axis = new THREE.Vector3().crossVectors(up, direction).normalize();
            const angle = Math.acos(up.dot(direction));

            ref.current.setRotationFromAxisAngle(axis, angle);
        }
    }, [start, end]);

    // if (log) {
    //     const { setPosition, addPathPoint } = useRobotStore();
    //     useFrame(() => {
    //         if (ref.current) {
    //             const target = end.clone().applyMatrix4(ref.current.matrixWorld);
    //             setPosition([target.x.toFixed(2), target.y.toFixed(2), target.z.toFixed(2)]);
    //             addPathPoint(target.clone());
    //         }
    //     });
    // }

    return (
        <mesh ref={ref}>
            <cylinderGeometry args={[0.2, 0.2, 1]} />
            <meshStandardMaterial color="gray" />
        </mesh>
    );
};
const Solid = ({ rotation, color = "lime" }) => {
    const ref = useRef(null);
    return (
        <mesh ref={ref} rotation={rotation}>
            <cylinderGeometry args={[0.5, 0.5, 1]} />
            <meshStandardMaterial color={color} />
        </mesh>
    );
};

const Robot = ({ position, target, onReachedTarget }) => {
    const baseRef = useRef();
    const joint1Ref = useRef();
    const joint2Ref = useRef();
    const joint3Ref = useRef();
    const wristRollRef = useRef();
    const wristPitchRef = useRef();
    const wristYawRef = useRef();

    const [angles, setAngles] = useState({ joint1: 0, joint2: 0, joint3: 0, wristRoll: 0, wristPitch: 0, wristYaw: 0 });
    const [isAtTarget, setIsAtTarget] = useState(false);

    // Move base towards target
    useFrame(() => {
        if (target) {
            const { x: tx, z: tz } = target;
            const { x, z } = baseRef.current.translation();
            // Move towards target
            const dx = tx - x;
            const dz = tz - z;
            const distance = Math.sqrt(dx * dx + dz * dz);

            if (distance > 0.5) {
                baseRef.current.applyImpulse({ x: dx * 0.2, y: 0, z: dz * 0.5 }, true);
            } else if (!isAtTarget) {
                console.log('reached target')
                setIsAtTarget(true);
                onReachedTarget();
            }
        }
    });

    // useFrame(() => {
    //     if (target != null) {
    // const newAngles = solveIK({ x: target.x, y: 8, z: target.z }, { roll: 0, pitch: 0, yaw: 0 }, 0.25, { l1: 2.0, l2: 2.0 });
    // setAngles(newAngles);
    //     }
    // });

    return (
        <RigidBody ref={baseRef} colliders="cuboid" position={position} mass={1000}>
            {/* Base */}
            <mesh position={[0, 0.25, 0]}>
                <boxGeometry args={[2, 0.5, 2]} />
                <meshStandardMaterial color="gray" />
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

            <RobotArm target={{ endEffectorX: target.x, endEffectorY: 10, endEffectorZ: target.z }} />

            {/* Robotic Arm */}
            {/* <group position={[0, 1.0, 0]}> */}
            {/* Joint 1 */}
            {/*     <group ref={joint1Ref} rotation={[0, angles.joint1, 0]}> */}
            {/*         <mesh position={[0, 1, 0]}> */}
            {/*             <cylinderGeometry args={[0.3, 0.3, 2]} /> */}
            {/*             <meshStandardMaterial color="red" /> */}
            {/*         </mesh> */}
            {/**/}
            {/* Joint 2 */}
            {/*         <group ref={joint2Ref} position={[0, 2, 0]} rotation={[angles.joint2, 0, 0]}> */}
            {/*             <mesh position={[0, 1, 0]}> */}
            {/*                 <cylinderGeometry args={[0.3, 0.3, 2]} /> */}
            {/*                 <meshStandardMaterial color="green" /> */}
            {/*             </mesh> */}
            {/**/}
            {/* Joint 3 */}
            {/*             <group ref={joint3Ref} position={[0, 2, 0]} rotation={[angles.joint3, 0, 0]}> */}
            {/*                 <mesh position={[0, 1, 0]}> */}
            {/*                     <cylinderGeometry args={[0.3, 0.3, 2]} /> */}
            {/*                     <meshStandardMaterial color="blue" /> */}
            {/*                 </mesh> */}
            {/**/}
            {/* Wrist Base */}
            {/*                 <group position={[0, 2, 0]}> */}
            {/* Wrist Joint 1 (Roll) */}
            {/*                     <group ref={wristRollRef} rotation={[0, 0, angles.wristRoll]}> */}
            {/*                         <mesh position={[0, 0.3, 0]}> */}
            {/*                             <cylinderGeometry args={[0.2, 0.2, 0.6]} /> */}
            {/*                             <meshStandardMaterial color="orange" /> */}
            {/*                         </mesh> */}
            {/**/}
            {/* Wrist Joint 2 (Pitch) */}
            {/*                         <group ref={wristPitchRef} position={[0, 0.6, 0]} rotation={[angles.wristPitch, 0, 0]}> */}
            {/*                             <mesh position={[0, 0.3, 0]}> */}
            {/*                                 <cylinderGeometry args={[0.2, 0.2, 0.6]} /> */}
            {/*                                 <meshStandardMaterial color="purple" /> */}
            {/*                             </mesh> */}
            {/**/}
            {/* Wrist Joint 3 (Yaw) */}
            {/*                             <group ref={wristYawRef} position={[0, 0.6, 0]} rotation={[0, 0, angles.wristYaw]}> */}
            {/*                                 <mesh position={[0, 0.3, 0]}> */}
            {/*                                     <cylinderGeometry args={[0.2, 0.2, 0.6]} /> */}
            {/*                                     <meshStandardMaterial color="yellow" /> */}
            {/*                                 </mesh> */}
            {/**/}
            {/* Gripper (for picking blocks) */}
            {/*                                 <group position={[0, 0.6, 0]}> */}
            {/*                                     <mesh position={[-0.2, 0, 0]}> */}
            {/*                                         <boxGeometry args={[0.1, 0.4, 0.2]} /> */}
            {/*                                         <meshStandardMaterial color="gray" /> */}
            {/*                                     </mesh> */}
            {/*                                     <mesh position={[0.2, 0, 0]}> */}
            {/*                                         <boxGeometry args={[0.1, 0.4, 0.2]} /> */}
            {/*                                         <meshStandardMaterial color="gray" /> */}
            {/*                                     </mesh> */}
            {/*                                 </group> */}
            {/*                             </group> */}
            {/*                         </group> */}
            {/*                     </group> */}
            {/*                 </group> */}
            {/*             </group> */}
            {/*         </group> */}
            {/*     </group> */}
            {/* </group> */}
        </RigidBody>
    );
};


// Environment with Blocks
const Environment = ({ onBlockSelect }) => {
    const blocks = [
        { id: 1, position: [3, 4.5, -2] },
        { id: 2, position: [-3, 2.5, 2] },
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
                <RigidBody key={id} type="fixed" colliders="cuboid" position={position}>
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
    const [target, setTarget] = useState({ x: 0, y: 0, z: 0 });

    const handleBlockSelect = (position) => {
        console.log("Box selected at position", position);
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
