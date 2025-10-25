from flask import Flask, render_template
import time

app = Flask(__name__, static_folder='static', template_folder='templates')

# Serve the main dashboard
@app.route('/')
def index():
    return render_template('index.html')  # your frontend HTML

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5500, debug=True)

