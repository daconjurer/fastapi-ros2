import pytest
from example_interfaces.srv import AddTwoInts

from fastapi_ros2.api.v1.models import Int64, RosModel, mirrors


class WrongType(RosModel):
    a: int
    b: Int64


class Missing(RosModel):
    a: Int64


class Reordered(RosModel):
    b: Int64
    a: Int64


class Defaulted(RosModel):
    a: Int64
    b: Int64 = 0


@pytest.mark.parametrize("model", [WrongType, Missing, Reordered, Defaulted])
def test_mirrors_rejects_a_model_that_drifts_from_its_message(model: type[RosModel]):
    with pytest.raises(TypeError, match=r"\(a: int64, b: int64\)$"):
        mirrors(AddTwoInts.Request)(model)
