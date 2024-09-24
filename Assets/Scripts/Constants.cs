using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.Scripts
{
    // Distance ENUM
    public enum DistanceTypes
    {
        Euclidean,
        Manhattan,
        Minkowski,
        Chebyshev
    }

    public class Constants
    {

        public bool CheckOverlaps(Vector3 ObjectPosition, Vector3 ExistingObjectPosition, float MinimumDistance)
        {
            // check two objects are further away than minimum distance.
            float ObjectDistance = Vector3.Distance(ObjectPosition, ExistingObjectPosition);
            if (MinimumDistance <= ObjectDistance)
            {
                return true;
            }

            return false;
        }
    }


}
