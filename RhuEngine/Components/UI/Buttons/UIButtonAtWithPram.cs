﻿using System;

using RhuEngine.WorldObjects;
using RhuEngine.WorldObjects.ECS;

using StereoKit;

namespace RhuEngine.Components
{
	[Category(new string[] { "UI\\Buttons" })]
	public class UIButtonAtWithPram<T> : UIComponent
	{
		[Default("Click Me")]
		public Sync<string> Text;

		public Sync<Vec3> TopLeft;

		public Sync<Vec2> Size;

		public Sync<T> Pram;

		public SyncDelegate<Action<T>> onClick;

		public override void RenderUI() {
			UI.PushId(Pointer.GetHashCode());
			if (UI.ButtonAt(Text.Value ?? "", TopLeft,Size)) {
				AddWorldCoroutine(() => onClick.Target?.Invoke(Pram));
			}
			UI.PopId();
		}
	}
}