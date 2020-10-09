extends Spatial

onready var file_dialog: Control = get_node("../UI/FileDialog")

export var point_cloud_scene: PackedScene

# Called when the node enters the scene tree for the first time.
func _ready():
	file_dialog.connect('file_selected', self, 'on_file_selected')
	file_dialog.popup()

func on_file_selected(path: String):
	var cloud = point_cloud_scene.instance()
	cloud.setSrcPath(path)

	get_parent().call_deferred('add_child', cloud)
	
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
