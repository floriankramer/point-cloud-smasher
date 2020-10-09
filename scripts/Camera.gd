extends Camera

var yaw = 0
var pitch = 0

export var bullet_scene: PackedScene
export var speed: float = 50
export var sensitivity: float = 1

var shoot_cooldown = 0

#func _ready():

func _unhandled_input(event):
	if event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.003 * sensitivity
		yaw = fmod(yaw, 2 * PI)
		pitch = clamp(pitch - event.relative.y * 0.003 * sensitivity, -PI * 0.45, PI * 0.45)
		var t = Transform.IDENTITY
		t = t.rotated(Vector3.RIGHT, pitch)
		t = t.rotated(Vector3.UP, yaw)
		t.origin = transform.origin
		transform = t

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	var motion: Vector3 = Vector3(0, 0, 0)
	if Input.is_action_pressed("move_forward"):
		motion -= transform.basis.z
	if Input.is_action_pressed("move_backward"):
		motion += transform.basis.z
	if Input.is_action_pressed("move_right"):
		motion += transform.basis.x
	if Input.is_action_pressed("move_left"):
		motion -= transform.basis.x
	if Input.is_action_pressed("jump"):
		motion += transform.basis.y
	if Input.is_action_pressed("crouch"):
		motion -= transform.basis.y
	
	motion = motion.normalized()
	transform.origin += motion * delta * speed
	
	if shoot_cooldown > 0:
		shoot_cooldown -= delta
	elif Input.is_action_pressed("shoot"):
		shoot_cooldown = 1
		var bullet: RigidBody = bullet_scene.instance()
		bullet.transform = transform
		bullet.apply_central_impulse(-transform.basis.z * 1500)
		get_parent().add_child(bullet)
