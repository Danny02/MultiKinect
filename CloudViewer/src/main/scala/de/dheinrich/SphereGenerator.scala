/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package de.dheinrich;

import darwin.geometrie.data._;
import darwin.renderer.opengl.GLSLType;
import darwin.util.math.base.vector._;

/**
 *
 * @author Daniel Heinrich <dannynullzwo@gmail.com>
 */

class SphereGenerator(tessFactor: Int) {
  private object SphereGenerator {
    val position = new Element(GLSLType.VEC3, "Position");
    val CUBE_SIDE_LENGTH = 1f;
  }
  import SphereGenerator._

  private var accIndex = 0

  if (tessFactor < 1) {
    throw new IllegalArgumentException();
  }

  private val sideCount = 2 + (tessFactor - 1)
  private val vertexCount = sideCount * sideCount * 6
  //vertexCount -= 2 * sideCount + 12;

  val buffer = new VertexBuffer(position, vertexCount);
  val indices = new Array[Int](6 * tessFactor * tessFactor * 2 * 3); //tf² = number of quads, * 2 = each quad has two tris, *3=each tris has 3 indices

  createSphere

  private def createSphere = {
    val half = CUBE_SIDE_LENGTH * 0.5f;
    //top
    createPlane(new Vector3(-half, half, -half), true);
    //bottom
    createPlane(new Vector3(-half, -half, -half), false);

    //front
    createSidePlaneX(new Vector3(-half, -half, half), true);
    //back
    createSidePlaneX(new Vector3(-half, -half, -half), false);

    //left
    createSidePlaneY(new Vector3(-half, -half, -half), true);
    //right
    createSidePlaneY(new Vector3(half, -half, -half), false);
  }

  private def createPlane(offset: ImmutableVector[Vector3], isTop: Boolean) = {
    for (i <- 0 until sideCount) {
      val side = new Vector3(0, 0, CUBE_SIDE_LENGTH);
      side.mul(i.toFloat / (sideCount - 1)).add(offset);
      createHoriziontalRow(side);
    }
    createQuadsForSide(isTop);
  }

  private def createSidePlaneX(offset: ImmutableVector[Vector3], isFront: Boolean) = {
    for (i <- 0 until sideCount) {
      val side = new Vector3(CUBE_SIDE_LENGTH, 0, 0);
      side.mul(i.toFloat / (sideCount - 1)).add(offset);
      createVerticalRow(side);
    }
    createQuadsForSide(isFront);
  }

  private def createSidePlaneY(offset: ImmutableVector[Vector3], isLeft: Boolean) = {
    for (i <- 0 until sideCount) {
      val side = new Vector3(0, 0, CUBE_SIDE_LENGTH);
      side.mul(i.toFloat / (sideCount - 1)).add(offset);
      createVerticalRow(side);
    }
    createQuadsForSide(isLeft);
  }

  private def createHoriziontalRow(offset: ImmutableVector[Vector3]) = {
    for (i <- 0 until sideCount) {
      addVertex(new Vector3(CUBE_SIDE_LENGTH, 0, 0), i, offset);
    }
  }

  private def createVerticalRow(offset: ImmutableVector[Vector3]) = {
    for (i <- 0 until sideCount) {
      addVertex(new Vector3(0, CUBE_SIDE_LENGTH, 0), i, offset);
    }
  }

  private def addVertex(pos: Vector3, i: Int, offset: ImmutableVector[Vector3]) = {
    val side = pos.mul(i.toFloat / (sideCount - 1)).add(offset);
    side.normalize();
    buffer.newVertex().setAttribute(position, side.getX(), side.getY(), side.getZ());
  }

  private def createQuadsForSide(isCW: Boolean) = {
    val created = sideCount * sideCount;
    val first = buffer.getVcount() - created;
    for (
      y <- 0 until (sideCount - 1);
      x <- 0 until (sideCount - 1)
    ) {
      val o = x + y * sideCount + first;
      addQuadIndices(!isCW, o, o + 1, o + sideCount + 1, o + sideCount);
    }
  }

  private def addQuadIndices(isCCW: Boolean, aa: Int, b: Int, cc: Int, d: Int) = {
    val a = if (isCCW) aa else cc
    val c = if (isCCW) cc else aa

    addIndex(a);
    addIndex(b);
    addIndex(c);

    addIndex(c);
    addIndex(d);
    addIndex(a);
  }

  private def addIndex(i: Int) = {
    indices(accIndex) = i;
    accIndex += 1
  }
}