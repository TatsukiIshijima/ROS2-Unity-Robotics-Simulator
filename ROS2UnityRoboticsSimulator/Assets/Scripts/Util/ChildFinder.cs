using System.Collections.Generic;
using UnityEngine;

namespace Util
{
    public static class ChildFinder
    {
        public static List<GameObject> GetAllChildren(GameObject parent)
        {
            var children = new List<GameObject>();
            GetAllChildrenRecursive(parent, children);
            return children;
        }

        private static void GetAllChildrenRecursive(GameObject parent, List<GameObject> children)
        {
            for (var i = 0; i < parent.transform.childCount; i++)
            {
                var child = parent.transform.GetChild(i);
                children.Add(child.gameObject);
                GetAllChildrenRecursive(child.gameObject, children);
            }
        }
    }
}