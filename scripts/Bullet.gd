extends RigidBody

export var lifetime: float = 10


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	lifetime -= delta
	if lifetime < 0:
		queue_free()
