﻿using System;

using RhuEngine.WorldObjects;
using RhuEngine.WorldObjects.ECS;

namespace RhuEngine.Components
{
	[Category(new string[] { "CoreEvents" })]
	public class AddSingleRefPram<T> : Component where T : class, IWorldObject
	{
		public readonly SyncRef<T> Ref;

		public readonly SyncDelegate<Action<T>> Target;

		[Exposed]
		public void Call() {
			Target.Target?.Invoke(Ref.Target);
		}
	}
}
