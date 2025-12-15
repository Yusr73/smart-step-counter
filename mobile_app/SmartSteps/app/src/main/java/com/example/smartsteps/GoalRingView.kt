package com.example.smartsteps

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import kotlin.math.min

class GoalRingView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyle: Int = 0
) : View(context, attrs, defStyle) {

    private var progress = 0f  // 0.0 → 1.0

    private val bgPaint = Paint().apply {
        color = 0xFFCCCCCC.toInt()
        style = Paint.Style.STROKE
        strokeWidth = 20f
        isAntiAlias = true
    }

    private val fgPaint = Paint().apply {
        color = 0xFF4CAF50.toInt()   // same green as JS
        style = Paint.Style.STROKE
        strokeWidth = 20f
        strokeCap = Paint.Cap.ROUND
        isAntiAlias = true
    }

    fun setProgress(value: Float) {
        progress = value.coerceIn(0f, 1f)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val size = min(width, height)
        val radius = size / 2f - 20f

        val cx = width / 2f
        val cy = height / 2f

        // Background circle
        canvas.drawCircle(cx, cy, radius, bgPaint)

        // Foreground arc (progress)
        val sweep = progress * 360f
        canvas.drawArc(
            cx - radius, cy - radius,
            cx + radius, cy + radius,
            -90f,
            sweep,
            false,
            fgPaint
        )
    }
}
