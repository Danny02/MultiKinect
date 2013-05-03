package de.dheinrich

import darwin.core.gui._
import javax.media.opengl.GLEventListener
import javax.inject.Inject
import darwin.renderer.opengl.VertexBO.VBOFactoy
import darwin.renderer.geometrie.packed.RenderMesh.RenderMeshFactory
import darwin.resourcehandling.dependencies.annotation.InjectBundle
import darwin.resourcehandling.shader.ShaderLoader
import darwin.renderer.shader.Shader
import darwin.renderer.geometrie.packed.RenderMesh
import darwin.core.timing.GameTime
import darwin.util.math.util.MatrixCache
import javax.media.opengl.GLAutoDrawable
import javax.media.opengl.DebugGL2
import javax.media.opengl.GL._
import darwin.renderer.shader.ShaderUniform
import darwin.util.math.composits.ViewMatrix
import com.google.common.base.Optional
import GuiceScalaInterop._
import java.util.concurrent.TimeUnit
import darwin.util.math.util.MatType

object GuiceScalaInterop {
  implicit def optional2Option[T](o: Optional[T]) = Option(o.orNull())
}

object TestRender{

  def main(args: Array[String]): Unit = {
    val debug = true;

    val client = Client.createClient(debug);

    val window = new ClientWindow(500, 500, false, client);
    window.startUp();

    val app = client.addGLEventListener(classOf[TestRender]);

    //        client.addMouseListener(a);
  }
}

class TestRender @Inject() (vboFactory: VBOFactoy, meshFactory: RenderMeshFactory) extends GLEventListener {
  //
  @InjectBundle(files = Array("sphere.frag", "sphere.vert"), prefix = ShaderLoader.SHADER_PATH_PREFIX)
  private var sphereShader: Shader = _
  //
  private var mesh: RenderMesh = _
  private var x, y, w, h = 0
  private var initialized = false;
  private val time = new GameTime();

  private val matrices = new MatrixCache();
  private val view = new ViewMatrix();
  private var timeUniform: Option[ShaderUniform] = None;

  def init(glad: GLAutoDrawable) = {
    glad.setGL(new DebugGL2(glad.getGL().getGL2()));
    val gl = glad.getGL().getGL2();
    import gl._

    view.loadIdentity();

    timeUniform = sphereShader.getUniform("time");
    matrices.addListener(sphereShader);

    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_ONE, GL_ONE);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    initialized = true;
  }

  def dispose(glad: GLAutoDrawable) {}

  def display(glad: GLAutoDrawable):Unit = {
    if (!initialized) {
      return;
    }

    val elepsed = time.update();
    val eInSeconds = elepsed.toFloat / TimeUnit.SECONDS.toNanos(1);
    timeUniform map(_.setData(time.getElapsedTime().toFloat))

    val gl = glad.getGL().getGL2();
    import gl._
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    if (sphereShader.isInitialized()) {
      if (mesh == null) {
        val gen = new SphereGenerator(200);
        val s = vboFactory.create(gen.buffer);
        mesh = meshFactory.create(sphereShader, GL_POINTS, null, s);
      }
      view.rotateEuler(0, 30 * eInSeconds, 0);
      view.rotateEuler(15 * eInSeconds, 0, 0);
      matrices.setView(view.clone().translate(0, 0, 5).inverse());

      sphereShader.updateUniformData();
      mesh.render();
    }
  }

  def reshape(glad: GLAutoDrawable, i: Int, i1: Int, width: Int, height: Int) = {
    w = width;
    h = height;
    val ratio = width.toFloat / (1 max height);
    matrices.getProjektion().perspective(50, ratio, 0.001, 1000);
    matrices.fireChange(MatType.PROJECTION);
  }
}