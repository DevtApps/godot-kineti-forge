@tool
extends EditorPlugin

## Editor entry point for the KinetiForge runtime addon.
##
## The vehicle classes use `class_name`, so Godot registers them directly
## from their scripts. Keeping this plugin intentionally small avoids adding
## editor-only dependencies to exported games.

func _enter_tree() -> void:
	pass

func _exit_tree() -> void:
	pass
