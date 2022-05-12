using System;
using System.Collections.Generic;
using System.Text;
using RhuEngine.WorldObjects;
using RhuEngine.WorldObjects.ECS;

using RNumerics;
using RhuEngine.Linker;
namespace RhuEngine.Components.Transform
{
	//single bone chain cyclic coordonate descent inverse kinematics
	public class IK : Component
	{
		public SyncRef<Entity> target;
		public SyncObjList<SyncRef<Entity>> joints;
		public SyncValueList<Vector3f> axis; //axis per joint that should bend

		public override void OnAttach() {
			base.OnAttach();
			target.Value = new Datatypes.NetPointer();
			var child = Entity.AddChild();
			for (var i = 0; i < 3; i++) {
				joints.GetValue(i).Target = null; // TODO;
				axis.GetValue(i).Value = new Vector3f(1, 0, 0);
			}
		}

		public override void Step() {
			for (var i = joints.Count - 1; i >= 0; i--) {
				var tempVec = Entity.LocalPosToGlobal(new Vector3f());

				//Rotate towards the Target														
				//(Ideally this could be done entirely in worldspace (instead of local space))
				var fromToQuat = Quaternionf.FromTo(
					joints[i].Target.GlobalPointToLocal(tempVec).Normalized,
					joints[i].Target.GlobalPointToLocal(target.Target.position).Normalized
				);
				joints[i].Target.rotation.SetValue(joints[i].Target.rotation * fromToQuat);

				//Find the rotation from here to the parent, and rotate the axis by it...
				//This ensures that you're always rotating with the hinge
				fromToQuat.SetFromTo(axis[i], joints[i].Target.rotation.Value.Inverse * axis.GetValue(i).Value);
				joints[i].Target.rotation.SetValue(joints[i].Target.rotation * fromToQuat);

				//Clamp to Joint Limits - Devious and relies on sensical computation of these values...
				//Seems like rotations range from -pi, pi... not the worst... but bad for clamps through there
				joints[i].Target.rotation.Value = Quaternionf.CreateFromEuler(
					joints[i].Target.rotation.Value.GetEuler().Clamp(-0.0174533f, 0.0174533f)); //none of this exists, please help!!!
			}
		}
	}
}