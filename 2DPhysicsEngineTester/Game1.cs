using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Flat;
using Flat.Graphics;
using Flat.Input;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using _2DPhysics;

namespace _2DPhysicsEngineTester
{
    public class Game1 : Game
    {
        private GraphicsDeviceManager graphics;
        private Screen screen;
        private Sprites sprites;
        private Shapes shapes;
        private Camera camera;

        private _2DWorld world;

        private List<Color> colors;
        private List<Color> outlineColors;

        private Vector2[] vertexBuffer;

        public Game1()
        {
            this.graphics = new GraphicsDeviceManager(this);
            this.graphics.SynchronizeWithVerticalRetrace = true;

            this.Content.RootDirectory = "Content";
            this.IsMouseVisible = true;
            this.IsFixedTimeStep = true;

            const double UpdatesPerSecond = 60d;
            this.TargetElapsedTime = TimeSpan.FromTicks((long)Math.Round((double)TimeSpan.TicksPerSecond / UpdatesPerSecond));
        }

        protected override void Initialize()
        {
            FlatUtil.SetRelativeBackBufferSize(this.graphics, 0.85f);

            this.screen = new Screen(this, 1280, 768);
            this.sprites = new Sprites(this);
            this.shapes = new Shapes(this);
            this.camera = new Camera(this.screen);
            this.camera.Zoom = 24;
            this.camera.GetExtents(out float left, out float right, out float bottom, out float top);

            float padding = MathF.Abs(left - right) * 0.10f; //padding is = to 10% of width

            this.world = new _2DWorld();
            this.colors = new List<Color>();
            this.outlineColors = new List<Color>();

            //add the ground
            if (!_2DBody.CreateBoxBody(right - left - padding * 2, 2f, new _2DVector(0, -10), 1f, true, 0.5f, out _2DBody groundBody, out string errorMessage))
            {
                throw new Exception(errorMessage);
            }
            this.world.AddBody(groundBody);

            this.colors.Add(Color.DarkGreen);
            this.outlineColors.Add(Color.White);

            base.Initialize();
        }

        protected override void LoadContent()
        {
            //base.LoadContent();
        }

        //recursive function for the game
        protected override void Update(GameTime gameTime)
        {
            FlatKeyboard Keyboard = FlatKeyboard.Instance;
            FlatMouse mouse = FlatMouse.Instance;

            Keyboard.Update();
            mouse.Update();

            if (mouse.IsLeftMouseButtonPressed())
            {
                float width = RandomHelper.RandomSingle(1f, 2f);
                float height = RandomHelper.RandomSingle(1f, 2f);

                _2DVector mouseWorldPosition = _2DConverter.To_2DVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));

                if (!_2DBody.CreateBoxBody(width, height, mouseWorldPosition, 2f, false, 0.6f, out _2DBody body, out string errorMessage))
                {
                    throw new Exception(errorMessage);
                }

                this.world.AddBody(body);
                this.colors.Add(RandomHelper.RandomColor());
                this.outlineColors.Add(Color.White);
            }

            if (mouse.IsRightMouseButtonPressed())
            {
                float radius = RandomHelper.RandomSingle(0.75f, 1.5f);

                _2DVector mouseWorldPosition = _2DConverter.To_2DVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));

                if (!_2DBody.CreateCircleBody(radius, mouseWorldPosition, 2f, false, 0.6f, out _2DBody body, out string errorMessage))
                {
                    throw new Exception(errorMessage);
                }

                this.world.AddBody(body);
                this.colors.Add(RandomHelper.RandomColor());
                this.outlineColors.Add(Color.White);
            }
            
            if (Keyboard.IsKeyAvailable)
            {
                if (Keyboard.IsKeyClicked(Keys.Escape))
                {
                    this.Exit();
                }

                if (Keyboard.IsKeyClicked(Keys.A))
                {
                    this.camera.IncZoom();
                }

                if (Keyboard.IsKeyClicked(Keys.Z))
                {
                    this.camera.DecZoom();
                }

                //float dx = 0f;
                //float dy = 0f;
                //float forceMagnitude = 48f;

                //if (Keyboard.IsKeyDown(Keys.Left)) { dx--; }
                //if (Keyboard.IsKeyDown(Keys.Right)) { dx++; }
                //if (Keyboard.IsKeyDown(Keys.Up)) { dy++; }
                //if (Keyboard.IsKeyDown(Keys.Down)) { dy--; }

                //if(!this.world.GetBody(0, out _2DBody body))
                //{
                //    throw new Exception("Could not find the body at the specified index");
                //}

                //if(dx != 0f || dy != 0)
                //{
                //    _2DVector forceDirection = _2DMath.Normalize(new _2DVector(dx, dy));
                //    _2DVector force = forceDirection * forceMagnitude;
                //    body.AddForce(force);
                //}
            }

            //
            this.world.Step(FlatUtil.GetElapsedTimeInSeconds(gameTime));

            
            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            this.screen.Set();
            this.GraphicsDevice.Clear(new Color(50, 60, 70));
            this.shapes.Begin(this.camera);

            for (int i = 0; i < this.world.BodyCount; i++)
            {
                if (!this.world.GetBody(i, out _2DBody body))
                {
                    throw new Exception("");
                }
                
                Vector2 position = _2DConverter.ToVector2(body.Position);

                if (body.shapeType == ShapeType.Circle)
                {
                    shapes.DrawCircleFill(position, body.Radius, 26, this.colors[i]);
                    shapes.DrawCircle(position, body.Radius, 26, this.outlineColors[i]);
                }
                else if (body.shapeType == ShapeType.Box)
                {
                    //shapes.DrawBox(position, body.Width, body.Height, Color.White);

                    _2DConverter.ToVector2Array(body.GetTransformedVerticies(), ref this.vertexBuffer);
                    shapes.DrawPolygonFill(this.vertexBuffer, body.triangles, this.colors[i]);
                    shapes.DrawPolygon(this.vertexBuffer, this.outlineColors[i]);
                }
            }


            this.shapes.End();
            this.screen.Unset();
            this.screen.Present(this.sprites);
            
            base.Draw(gameTime);
        }

        //keep bodies within the bounds of the camera
        private void WrapScreen()
        {
            this.camera.GetExtents(out Vector2 camMin, out Vector2 camMax);

            float viewWidth = camMax.X - camMin.X;
            float viewHeight = camMax.Y - camMin.Y;

            for (int i = 0; i < this.world.BodyCount; i++)
            {
                if (!this.world.GetBody(i, out _2DBody body))
                {
                    throw new Exception();
                }

                if (body.Position.X < camMin.X) { body.MoveTo(body.Position + new _2DVector(viewWidth, 0f)); }
                if (body.Position.X > camMax.X) { body.MoveTo(body.Position - new _2DVector(viewWidth, 0f)); }
                if (body.Position.Y < camMin.Y) { body.MoveTo(body.Position + new _2DVector(0f, viewHeight)); }
                if (body.Position.Y > camMax.Y) { body.MoveTo(body.Position - new _2DVector(0f, viewHeight)); }

            }
        }
    }
}
