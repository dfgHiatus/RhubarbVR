using System.Linq;

using RhuEngine.WorldObjects.ECS;

using RNumerics;

namespace RhuEngine.Components
{
	public class EasyIK : Component
	{
		public int numberOfJoints = 2;
		public Matrix ikTarget;
		public int iterations = 10;
		public float tolerance = 0.05f;
		private Matrix[] jointTransforms;
		private Vector3f startPosition;
		private Vector3f[] jointPositions;
		private float[] boneLength;
		private float jointChainLength;
		private float distanceToTarget;
		private Quaternionf[] startRotation;
		private Vector3f[] jointStartDirection;
		private Quaternionf ikTargetStartRot;
		private Quaternionf lastJointStartRot;

		public Matrix poleTarget;

		public bool debugJoints = true;
		public bool localRotationAxis = false;

		// Remove this if you need bigger gizmos:
		public float gizmoSize = 0.05f;
		public bool poleDirection = false;
		public bool poleRotationAxis = false;

		public override void OnLoaded() {
			Reload();
		}

		private void Reload() {
			// Let's set some properties
			jointChainLength = 0;
			jointTransforms = new Matrix[numberOfJoints];
			jointPositions = new Vector3f[numberOfJoints];
			boneLength = new float[numberOfJoints];
			jointStartDirection = new Vector3f[numberOfJoints];
			startRotation = new Quaternionf[numberOfJoints];
			ikTargetStartRot = ikTarget.Rotation;

			// TODO Get current gameobject
			var current = Entity;

			// For each bone calculate and store the lenght of the bone
			for (var i = 0; i < jointTransforms.Length; i += 1) {
				jointTransforms[i] = current.GlobalTrans;
				// If the bones lenght equals the max lenght, we are on the last joint in the chain
				if (i == jointTransforms.Length - 1) {
					lastJointStartRot = current.rotation;
					return;
				}
				// Store length and add the sum of the bone lengths
				else {
					boneLength[i] = current.position.Value.Distance(current.children[0].position);
					jointChainLength += boneLength[i];

					jointStartDirection[i] = current.children[0].position.Value - current.position;
					startRotation[i] = current.rotation;
				}
				// Move the iteration to next joint in the chain
				current = current.children[0];
			}
		}

		void PoleConstraint() {
			if (poleTarget != null && numberOfJoints < 4) {
				// Get the limb axis direction
				var limbAxis = (jointPositions[2] - jointPositions[0]).Normalized;

				// Get the direction from the root joint to the pole target and mid joint
				var poleDirection = (poleTarget.Translation - jointPositions[0]).Normalized;
				var boneDirection = (jointPositions[1] - jointPositions[0]).Normalized;

				// Ortho-normalize the vectors
				// TODO Implement a Gram-Schmidt Orthonormaliztion function
				// Vector3f.OrthoNormalize(ref limbAxis, ref poleDirection);
				// Vector3f.OrthoNormalize(ref limbAxis, ref boneDirection);
				var temp = Vector3f.Cross(limbAxis, poleDirection);
				poleDirection = Vector3f.Cross(temp, limbAxis);
				temp = Vector3f.Cross(limbAxis, boneDirection);
				boneDirection = Vector3f.Cross(temp, limbAxis);

				// Calculate the angle between the boneDirection vector and poleDirection vector
				var angle = Quaternionf.FromToRotation(boneDirection, poleDirection);

				// Rotate the middle bone using the angle
				jointPositions[1] = (angle * (jointPositions[1] - jointPositions[0])) + jointPositions[0];
			}
		}

		void Backward() {
			// Iterate through every position in the list until we reach the start of the chain
			for (var i = jointPositions.Length - 1; i >= 0; i -= 1) {
				// The last bone position should have the same position as the ikTarget
				if (i == jointPositions.Length - 1) {
					jointPositions[i] = ikTarget.Translation;
				}
				else {
					jointPositions[i] = jointPositions[i + 1] + (jointPositions[i] - jointPositions[i + 1]).Normalized * boneLength[i];
				}
			}
		}

		void Forward() {
			// Iterate through every position in the list until we reach the end of the chain
			for (var i = 0; i < jointPositions.Length; i += 1) {
				// The first bone position should have the same position as the startPosition
				if (i == 0) {
					jointPositions[i] = startPosition;
				}
				else {
					jointPositions[i] = jointPositions[i - 1] + (jointPositions[i] - jointPositions[i - 1]).Normalized * boneLength[i - 1];
				}
			}
		}

		private void SolveIK() {
			// Get the jointPositions from the joints
			for (var i = 0; i < jointTransforms.Length; i += 1) {
				jointPositions[i] = jointTransforms[i].Translation;
			}
			// Distance from the root to the ikTarget
			distanceToTarget = jointPositions[0].DistanceSquared(ikTarget.Translation);

			// IF THE TARGET IS NOT REACHABLE
			if (distanceToTarget > jointChainLength) {
				// Direction from root to ikTarget
				var direction = ikTarget.Translation - jointPositions[0];

				// Get the jointPositions
				for (var i = 1; i < jointPositions.Length; i += 1) {
					jointPositions[i] = jointPositions[i - 1] + direction.Normalized * boneLength[i - 1];
				}
			}
			// IF THE TARGET IS REACHABLE
			else {
				// Get the distance from the leaf bone to the ikTarget
				var distToTarget = jointPositions[jointPositions.Length - 1].DistanceSquared(ikTarget.Translation);
				var counter = 0;
				// While the distance to target is greater than the tolerance let's iterate until we are close enough
				while (distToTarget > tolerance) {
					startPosition = jointPositions[0];
					Backward();
					Forward();
					counter += 1;
					// After x iterations break the loop to avoid an infinite loop
					if (counter > iterations) {
						break;
					}
				}
			}
			// Apply the pole constraint
			PoleConstraint();

			// Apply the jointPositions and rotations to the joints
			for (var i = 0; i < jointPositions.Length - 1; i += 1) {
				jointTransforms[i].Translation = jointPositions[i];
				var targetRotation = Quaternionf.FromToRotation(jointStartDirection[i], jointPositions[i + 1] - jointPositions[i]);
				var temp = targetRotation * startRotation[i];
				jointTransforms[i].Rotation = temp[0];
			}
			// Let's constrain the rotation of the last joint to the IK target and maintain the offset.
			var offset = lastJointStartRot * ikTargetStartRot.Inverse;
			jointTransforms.Last().Rotation = ikTarget.Rotation * offset;
		}

		void Update() 
		{
			SolveIK();
		}
	}
}
