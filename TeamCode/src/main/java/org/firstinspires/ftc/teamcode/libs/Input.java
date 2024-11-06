package org.firstinspires.ftc.teamcode.libs;

public final class Input {
	private boolean state;
	private boolean previous;

	public Input() {
		this(false);
	}

	public Input(boolean state) {
		this.state = state;
		this.previous = state;
	}

	public void update(boolean state) {
		this.previous = this.state;
		this.state = state;
	}

	public boolean wasChanged() {
		return this.state != this.previous;
	}

	public boolean wasPressed() {
		return this.state && !this.previous;
	}

	public boolean wasReleased() {
		return !this.state && this.previous;
	}

	public boolean isDown() {
		return this.state;
	}
}
