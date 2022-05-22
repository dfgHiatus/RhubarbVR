﻿using RhuEngine.WorldObjects;
using RhuEngine.WorldObjects.ECS;

using RNumerics;
using RhuEngine.Linker;

namespace RhuEngine.Components
{
	[UpdateLevel(UpdateEnum.Movement)]
	[Category(new string[] { "Transform" })]
	public class LookAt : Component
	{
		[OnChanged(nameof(BindAtPoint))]
		public readonly SyncRef<IValueSource<Vector3f>> LookAtPoint;

		public readonly Linker<Quaternionf> Driver;

		public readonly Sync<Quaternionf> offset;

		IValueSource<Vector3f> _lastLookAtPoint;

		private void BindAtPoint() {
			if (_lastLookAtPoint is not null) {
				_lastLookAtPoint.Changed -= Compute;
			}
			if (LookAtPoint.Target is not null) {
				_lastLookAtPoint.Changed += Compute;
				Compute(null);
			}
			_lastLookAtPoint = LookAtPoint.Target;
		}

		
		public void ComputeOutput() {
			Compute(null);
		}

		private void Compute(IChangeable changeable) {
			if (Driver.Linked) {
				if (LookAtPoint.Target is not null) {
					Driver.LinkedValue = Quaternionf.LookAt(Entity.GlobalTrans.Translation,LookAtPoint.Target.Value) * offset.Value;
				}
			}
		}



		public override void OnAttach() {
			base.OnAttach();
			offset.Value = Entity.rotation.Value;
			Driver.SetLinkerTarget(Entity.rotation);
		}
	}
}